#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/adc.h"
#include "drivers/buttons.h"
#include "driverlib/comp.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "grlib/grlib.h"

#include "drivers/cfal96x64x16.h"

#include "utils/uartstdio.h"

#define LED_ON          20000
#define LED_OFF         2000000
#define MOTOR_OFF       0x0
#define STEPS_PER_REV   48 
#define START_RPM       60
#define RPM_MAX         300
#define RPM_MIN         1

tContext g_sContext;
tRectangle g_sRect;
volatile uint32_t g_vui32Loop;
int32_t motorStep = 0;
int32_t RPM = 60;
bool followMotor = false;
bool reverseStepMotor = false;
bool toggleMotor = true;
bool killProgram = false;

void print_menu(void);
void process_menu(uint32_t local_char);
void MotorSteps(void);
void UARTIntHandler(void);
void UARTSend(const char *pui8Buffer, uint32_t ui32Count);
void Timer0IntHandler(void);
void UARTIntHandler(void);


int
main(void)
{
  //
  // Enable lazy stacking for interrupt handlers.  This allows floating-point
  // instructions to be used within interrupt handlers, but at the expense of
  // extra stack usage.
  //
  FPULazyStackingEnable();
  
  //
  // Set the clocking to run directly from the crystal.
  //
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_16MHZ);
  
  IntMasterDisable();
  
  //
  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
  {
  }
  
  GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
  
  //
  // Initialize the display driver.
  //
  CFAL96x64x16Init();
  
  //
  // Initialize the graphics context.
  //
  GrContextInit(&g_sContext, &g_sCFAL96x64x16);
  
  //
  // Set GPIO A0 and A1 as UART pins.
  //
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_2| 
                        GPIO_PIN_3);
  GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0| GPIO_PIN_1| GPIO_PIN_2| 
               GPIO_PIN_3, 0xC | 0x6 | 0x3 | 0x9);
  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  
  IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
  
  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_6);
  ADCSequenceDisable(ADC0_BASE, 1);
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH5 | ADC_CTL_IE
                           | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 1);
  
  //
  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  
  //
  // Configure the two 32-bit periodic timers.
  //
  TimerConfigure( TIMER0_BASE, TIMER_CFG_PERIODIC);
  
  TimerLoadSet (TIMER0_BASE, TIMER_A, SysCtlClockGet() / RPM);
  
  
  //
  // Setup the interrupts for the timer timeouts.
  //
  
  IntEnable(INT_TIMER0A);
  
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  
  TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
  //
  // Enable the timers.
  // 
  TimerEnable(TIMER0_BASE, TIMER_A);
  TimerEnable(TIMER1_BASE, TIMER_A);
  
  //
  // Black rectangle to clear screen
  //
  g_sRect.i16XMin = 0;
  g_sRect.i16YMin = 0;
  g_sRect.i16XMax = GrContextDpyWidthGet (&g_sContext) -1;
  g_sRect.i16YMax = 59;
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &g_sRect);
  
  //
  // Splash screen displayed for a few seconds
  //
  
  GrContextForegroundSet(&g_sContext, ClrWhite);
  GrContextFontSet(&g_sContext,  g_psFontFixed6x8);
  GrStringDrawCentered(&g_sContext, "Zach and Andrew from A-Z", -1,
                       GrContextDpyWidthGet(&g_sContext) / 2, 35, true);
  
  for (g_vui32Loop = 0; g_vui32Loop <LED_ON * 60; g_vui32Loop++)
  {
  }
  
  print_menu();
  
  IntMasterEnable();
  
  while(!killProgram)
  {
    
    //
    // Fill the top part of the screen with blue to create the banner.
    //
    g_sRect.i16XMin = 0;
    g_sRect.i16YMin = 0;
    g_sRect.i16XMax = GrContextDpyWidthGet(&g_sContext) - 1;
    g_sRect.i16YMax = 9;
    GrContextForegroundSet(&g_sContext, ClrDarkOrange);
    GrRectFill(&g_sContext, &g_sRect);
    
    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&g_sContext, ClrWhite);
    
    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&g_sContext,  g_psFontFixed6x8);
    GrStringDrawCentered(&g_sContext, "Starting from the Bottom", -1,
                         GrContextDpyWidthGet(&g_sContext) / 2, 4, true);
    
    //
    // Turn on the LED.
    //
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2); 
    
    //
    // Delay for a bit.
    //
    for (g_vui32Loop = 0 ; g_vui32Loop < LED_ON; g_vui32Loop++)
    {
    }
    
    // 
    // Turn off the LED.
    //
    GPIOPinWrite( GPIO_PORTG_BASE, GPIO_PIN_2, 0);
    
    //
    // Delay for a bit.
    //
    for (g_vui32Loop = 0 ; g_vui32Loop < LED_OFF; g_vui32Loop++)
    {
    }
    //
    // Toggle LED
    //
    
    for (g_vui32Loop = 0 ; g_vui32Loop < LED_ON * 40; g_vui32Loop++)
    {
    }
  }
}

void
MotorSteps(void)
{
  const int full_step[5] = {0xC, 0x6, 0x3, 0x9};
  
  if(!toggleMotor)
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
                 GPIO_PIN_3, MOTOR_OFF);
  else
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
                 GPIO_PIN_3, full_step[motorStep]);
}


void 
print_menu(void)
{
  char screen[] = "Menu Selection:\n\r"
    "P = Print this Menu\r\n"
      "R = Reverse Motor\r\n"
	"T = Toggle Motor\r\n"
          "F = Motor Follow\r\n"
            "+ = Increase Speed\r\n"
              "- = Decrease Speed\r\n";
  UARTSend(screen, strlen(screen));
}

void 
process_menu (uint32_t local_char)
{
  char RPMStr[30];
  UARTCharPutNonBlocking(UART0_BASE, local_char);
  
  switch (toupper(local_char))
  {
  case 'P':
    print_menu();
    break;
  case 'K':
    killProgram = !killProgram;
    break;
  case 'R':
    reverseStepMotor = !reverseStepMotor;
    break;
  case 'T':
    toggleMotor = !toggleMotor;
    break;
  case 'F':
    followMotor = !followMotor;
    break;
  case '+':
    RPM++;
    if(RPM > RPM_MAX)
      RPM = 300;
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() * 60) / (RPM * 48));
    sprintf(RPMStr, "RPM = %d\n\r", RPM);
    UARTSend(RPMStr, strlen(RPMStr));
    break;
  case '-':
    RPM--;
    if(RPM < RPM_MIN)
      RPM = 1;
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet() * 60) / (RPM * 48));
    sprintf(RPMStr, "RPM = %d\n\r", RPM);
    UARTSend(RPMStr, strlen(RPMStr));
    break;
  default:
    UARTSend("Tu eres muy estupido\r\n", 22);
  }
}
  
  void 
    UARTSend(const char *pui8Buffer, uint32_t ui32Count)
    {
      //
      // Loop while there are more characters to send.
      //
      while(ui32Count--)
      {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE, *pui8Buffer++);
      }
    }
  
  //*****************************************************************************
  //
  // The Timer 0 interrupt handler.
  //
  //*****************************************************************************
  void
    Timer0IntHandler(void)
    {
      static int steps_moved;
      int steps_desired;
      static int followCount = 0;
      static uint32_t ADC_Home;
      uint32_t ADC_Cur;
      
      TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

      MotorSteps();
      
      if(!followMotor)
      {
        if(!reverseStepMotor)
        {
	  motorStep++;
	  if (motorStep > 3)
            motorStep = 0;
        }
        
        if(reverseStepMotor)
        {
	  motorStep--;
	  if(motorStep < 0)
            motorStep = 3;
        }
      }

      if(followMotor)
      {
        ADCIntClear(ADC0_BASE, 1);
        
        ADCProcessorTrigger(ADC0_BASE, 1);
        
        while(!ADCIntStatus(ADC0_BASE, 1, false));
        
        ADCSequenceDataGet(ADC0_BASE, 1, &ADC_Cur);

          if(followCount == 0)
          {
            steps_moved = 0;
            ADC_Home = ADC_Cur;
            followCount = 1;
            TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 60);
          }
          else 
          {
            steps_desired = ((uint32_t) ADC_Cur -  (uint32_t) ADC_Home);
            steps_desired = (steps_desired * 48) / 4095;
            if(steps_desired > steps_moved)
            {
              steps_moved++;
              
              motorStep++;
              if (motorStep > 3)
                motorStep = 0;
            }
            else if (steps_desired < steps_moved)
            {
              steps_moved--;
              
              motorStep--;
              if(motorStep < 0)
                motorStep = 3;
            }
        }
      }
    }
  
  //*****************************************************************************
  //
  // The UART interrupt handler.
  //
  //*****************************************************************************
  void 
    UARTIntHandler(void)
    {
      uint32_t ui32Status;
      char local_char;
      
      //
      // Get the interrrupt status.
      //
      ui32Status = UARTIntStatus(UART0_BASE, true);
      
      //
      // Clear the asserted interrupts.
      //
      UARTIntClear(UART0_BASE, ui32Status);
      
      local_char = UARTCharGetNonBlocking(UART0_BASE);
      
      process_menu(local_char);
      
      IntMasterEnable();
    }
  