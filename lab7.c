#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "limits.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/adc.h"
#include "drivers/buttons.h"
#include "driverlib/comp.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "grlib/grlib.h"

#include "drivers/cfal96x64x16.h"

#include "utils/uartstdio.h"

#define LED_ON          20000
#define LED_OFF         2000000

// globals
tContext g_sContext;
tRectangle g_sRect;

volatile uint32_t g_vui32Loop;

bool debugOn = false;
bool killProgram = false;

char waveState;
char local_char;

static unsigned char waveValue[2];
//bool squareOn = false;

// Function prototypes
int i2c_write(uint32_t i2c_base_addr, unsigned char dev_addr, unsigned num_char, 
              unsigned char *write_buf);
static unsigned long WaitI2CDone(unsigned int long ulBase);
static unsigned long ResetI2C(uint32_t i2c_base);
void print_menu(void);
void process_menu(uint32_t local_char);
void splashScreen(void);
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
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
  
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
  {
  }
  
  GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
  
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
  {
  }
  
  GPIOPinConfigure(GPIO_PA6_I2C1SCL);
  GPIOPinConfigure(GPIO_PA7_I2C1SDA);
  
  GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
  GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
  
  I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
  I2CMasterSlaveAddrSet(I2C1_BASE, 0x60, false);
  
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
  
  //
  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  
  //
  // Configure the two 32-bit periodic timers.
  //
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  
  TimerLoadSet (TIMER0_BASE, TIMER_A, SysCtlClockGet() / 4095);
  
  
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
  
  //
  // Black rectangle to clear screen
  //
  g_sRect.i16XMin = 0;
  g_sRect.i16YMin = 0;
  g_sRect.i16XMax = GrContextDpyWidthGet (&g_sContext) -1;
  g_sRect.i16YMax = 59;
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &g_sRect);
  
  print_menu();
  waveValue[0] = waveValue[1] = 0;
  
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
    GrStringDrawCentered(&g_sContext, "Lab 7", -1,
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
print_menu(void)
{
  char screen[] = "Menu Selection:\n\r"
    "P = Print this Menu\r\n"
      "W = Sawtooth\r\n"
	    "T = Triangle\r\n"
          "Q = Square\r\n"
            "K = Kill program\r\n"
              "D = Debug\r\n"
			    "S = Splash Screen\r\n";
  UARTSend(screen, strlen(screen));
}

void 
process_menu (uint32_t local_char)
{
  UARTCharPutNonBlocking(UART0_BASE, local_char);
  
  switch (toupper(local_char))
  {
  case 'P':
    print_menu();
    break;
  case 'K':
    killProgram = !killProgram;
    break;
  case 'W':
    waveState = 'W';
    break;
  case 'T':
    waveState = 'T';
    break;
  case 'Q':
    waveState = 'Q';
    break;
  case 'D':
    waveState = 'D';
    debugOn = !debugOn;
    break;
  case 'S':
    splashScreen();
	break;
  default:
    UARTSend("Tu eres muy estupido\r\n", 22);
  }
}

void
splashScreen(void)
{
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
  
  //
  // Black rectangle to clear screen
  //
  g_sRect.i16XMin = 0;
  g_sRect.i16YMin = 0;
  g_sRect.i16XMax = GrContextDpyWidthGet (&g_sContext) -1;
  g_sRect.i16YMax = 59;
  GrContextForegroundSet(&g_sContext, ClrBlack);
  GrRectFill(&g_sContext, &g_sRect);
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
// 
//
//*****************************************************************************
static unsigned long ResetI2C(uint32_t i2c_base){
    
    SysCtlPeripheralReset(i2c_base);
    
    SysCtlDelay(40);
    
    // while (! SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {}
    
    return I2CMasterErr(i2c_base);
}
//**************************************************************************
//
// WaitI2CDone()
//
// Arguments:
//   ulBase -- > the base address of the i2c peripheral being used
//               i.e. I2C2_BASE
//*****************************************************************************
static unsigned long WaitI2CDone(unsigned int long ulBase){
    const unsigned threshold = (UINT_MAX-10);
    unsigned ctr = 0;
    // Wait until done transmitting
    while(I2CMasterBusy(ulBase))
    {
      ctr++;
      if (ctr >= threshold)
      {
          ResetI2C(ulBase);
          break;
      }
    }
    // Return I2C error code
    return I2CMasterErr(ulBase);
}

//*****************************************************************************
//
// I2C_write()
// 
// Arguments:
//   i2c_base_address -- > the base address of the i2c peripheral being used
//                         i.e. I2C2_BASE
//   dev_addr --> the I2C address of the device being written to. This value
//                should always be in the range 0x1 - 0x7F, or 0x0 for broadcast.
//   num_char --> the number of characters to be written to the I2C bus
//                exclusive of the address byte.  A call to i2c_write with
//                num_char = 4 will result in 5 bytes total being written
//                to the bus, the address byte and 4 data bytes.
//   write_buf --> a pointer to the bytes to be written to the I2C bus.
//                 this buffer must contain at least bytes num_char of data.
//                 Any buffer data beyond num_char bytes will be ignored. 
//*****************************************************************************
int i2c_write(uint32_t i2c_base_addr, 
              unsigned char dev_addr, 
              unsigned num_char, 
              unsigned char *write_buf)
{
  int rv = 0;
  if (num_char == 0)
    return rv;
  
  // Make sure 
  WaitI2CDone(i2c_base_addr);  
  if (num_char == 1)
  {
    // Set the slave address
    // recieve = false
    I2CMasterSlaveAddrSet(i2c_base_addr, dev_addr , false);
    // 
    I2CMasterDataPut(i2c_base_addr, write_buf[0]);
    
    // Send the command to initiate the read
    I2CMasterControl(i2c_base_addr, I2C_MASTER_CMD_SINGLE_SEND);
    // Wait for that transmission to finish
    WaitI2CDone(i2c_base_addr);
  }
  else
  {
    int burst_cont = (num_char - 2);
    int index = 0;
    
    // Set the slave address
    // recieve = false
    I2CMasterSlaveAddrSet(i2c_base_addr, dev_addr , false);
    // Tell the slave to start reading
    I2CMasterDataPut(i2c_base_addr, write_buf[index++]);
    // Start Burst
    I2CMasterControl(i2c_base_addr, I2C_MASTER_CMD_BURST_SEND_START);
    // Wait for that transmission to finish
    WaitI2CDone(i2c_base_addr);
    
    while (burst_cont > 0)
    {
      // Tell the slave to start reading
      I2CMasterDataPut(i2c_base_addr, write_buf[index++]);
      // Continue Burst
      I2CMasterControl(i2c_base_addr, I2C_MASTER_CMD_BURST_SEND_CONT);
      // Wait for that transmission to finish
      WaitI2CDone(i2c_base_addr);
      burst_cont--;
    }
    
    // Tell the slave to start reading
    I2CMasterDataPut(i2c_base_addr, write_buf[index++]);
    // Finish Burst
    I2CMasterControl(i2c_base_addr, I2C_MASTER_CMD_BURST_SEND_FINISH);
    // Wait for that transmission to finish
    WaitI2CDone(i2c_base_addr);
  } // case num_char > 1
  
  return(rv);
}

  //*****************************************************************************
  //
  // The Timer 0 interrupt handler.
  //
  //*****************************************************************************
  void
    Timer0IntHandler(void)
    {   
      TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
      static bool wavePhase = true;
      static int waveCounter = 0;
	  
      switch (toupper(waveState))
      {
      case 'W':
        if (waveValue[1] == 0xFF && waveValue[0] == 0x0F)
        {
          waveValue[0] = waveValue[1] = 0;
         // char shit[25] = "squares:  ";
         // sprintf(shit, "%d ", squareCounter);
         // UARTSend(shit, sizeof(shit) - 1);
        }
        
        if (waveValue[1] == 0xFF && waveValue[0] < 0x0F)
        {
          waveValue[1] = 0x00;
          waveValue[0]++;
        }
        
        if (waveValue[1] < 0xFF)
          waveValue[1]++;
        // squareCounter++;
        break;
      case 'Q':
        //squareOn = !squareOn;
        
        waveCounter ++;
		
		if(waveCounter == 4080)
        {
          waveCounter = 0;
          wavePhase = !wavePhase;
        }
		
        if (waveCounter < 4080 && wavePhase)
        {
        waveValue[1] = 0x00;
        waveValue[0] = 0x00;
        
        }
        if (waveCounter < 4080 && !wavePhase)
        {
        waveValue[1] = 0xFF;
        waveValue[0] = 0x0F;
        }

        break;
        
      case 'T':
	  
	    waveCounter ++;
		
		if(waveCounter == 4080)
        {
          waveCounter = 0;
          wavePhase = !wavePhase;
        }
		
        if(waveCounter < 4080 && wavePhase)
        {
          if (waveValue[1] == 0xFF && waveValue[0] < 0x0F)
          {
            waveValue[1] = 0x00;
            waveValue[0]++;
          }
       
          if (waveValue[1] < 0xFF)
            waveValue[1]++;
        }
        
        if (waveCounter < 4080 && !wavePhase)
		{
	      if (waveValue[1] == 0x00 && waveValue[0] > 0x00)
          {
            waveValue[1] = 0xFF;
            waveValue[0]--;
          }
        
          if (waveValue[1] > 0x00)
            waveValue[1]--;
		}			
        
        break;
      case 'D':
        if (debugOn)
          TimerLoadSet (TIMER0_BASE, TIMER_A, SysCtlClockGet() / 20);
        if (!debugOn)
          TimerLoadSet (TIMER0_BASE, TIMER_A, SysCtlClockGet() / 4095);
        break;
      }
      i2c_write(I2C1_BASE, 0x60, 2, waveValue);
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