/*********************************************************************************/
//              Lab 5 - Serial Communication and SPI Interface           	 
//                                                                        	 
// Description: Using the MSP-EXP430FG4618 Development Tool establish a      	 
//           	data exchange between the MSP430FG4618 and MSP430F2013           
//           	devices using the SPI mode. The MSP430FG4618 uses the USCI       
//           	module while the MSP430F2013 uses the USI module.                
//              MSP430FG4618 communicates with PC via RS232 module using                                                          
//              USCI Serial Communication peripheral interface. This program     
//		takes user prompts the user to input a choice to turn ON or OFF  
//		the LED3 located on MSP430F2013. The user choice is communicated 
//		to MSP430FG4618 (master) via USCI serial interface and the 	 
//		corresponding action is communicated to MSP430F2013(slave) via 	 
//		SPI. Based on the user choice, MSP430F2013 will turn ON or OFF	 
//		the LED3. This is the salve code that runs on MSP430F2013.     
//                                                                        	 
//                                 Slave                    Master                  
//                              MSP430F2013              MSP430FG4618             
//                          -----------------          -----------------          
//                         |              XIN|-    /|\|              XIN|-        
//                         |                 |      | |                 |  32kHz xtal
//                         |             XOUT|-     --|RST          XOUT|-           
//                         |                 |        |                 |            
//                   LED <-|P1.0             |        |                 |            
//                         |         BF /P1.4|------->|P3.0/BF          |  
//                         |         SDI/P1.7|<-------|P3.1/UCB0SIMO    | 
//                         |         SDO/P1.6|------->|P3.2/UCB0SOMI    |  
//                         |        SCLK/P1.5|<-------|P3.3/UCB0CLK     | 
//                                                                        
// Instructions:                                                          
//                                                                        
// 1. Set the following parameters in hyperterminal                       
//						Port :		COM1      
//						Baud rate :	19200     
//						Data bits:	8         
//						Parity:		None      
//						Stop bits:	1         
//						Flow Control:	None	  
// 2. This lab requires to configure the USI module of MSP430F2013        
//    as slave and MSP430FG4618 as master in SPI mode.                    
// 3. Connect the following jumpers on header 1 (H1) on the experimenter's  
//    board.  [1-2], [3-4], [5-6], [7-8]                                  
/*		   H1	
		_______
	      1|-------|2
	      3|-------|4
	      5|-------|6
	      7|-------|8		
	       |_______|

      Jumper must be present on	PWR1, PWR2 and JP2. 

// 4. Download and run this code by the connecting the FET debugger to 
//    JTAG2 on the experimenter's board.
// 5. Make sure the device selected is MSP430F2013 in the General Options of IAR.

**************************************************************************/


#include "msp430x20x3.h"

#define LED_ON_STATE    0x31  // character '1'
#define LED_OFF_STATE   0x30  // character '0'
#define LED_NUL_STATE   0x00  // character NULL - used for dummy write operation
                
#define LED_ON          0x01
#define LED_OFF         0x00

#define SET_BUSY_FLAG()   P1OUT |= 0x10;
#define RESET_BUSY_FLAG() P1OUT &= ~0x10;

#define SET_LED()       P1OUT |= 0x01;
#define RESET_LED()     P1OUT &= ~0x01;

unsigned char LEDState;
unsigned char NextState;

void SPISetup(void)
{
  
  USICTL0 |= USISWRST;      //Set UCSWRST -- needed for re-configuration process
  USICTL0 |= USIPE5 + USIPE6 + USIPE7 + USIOE; // SCLK-SDO-SDI port enable,MSB first   
  USICTL1 = USIIE;          // USI  Counter Interrupt enable                              
  USICTL0 &= ~USISWRST;     // **Initialize USCI state machine**
}



void InitComm(void)
{ 
 USICNT = 8;                 // Load bit counter, clears IFG
 USISRL = LEDState;          // set LED state
 RESET_BUSY_FLAG();          // reset busy flag
} 

void LEdInit(unsigned char state)
{
  if (state == LED_OFF_STATE)
  {
    RESET_LED();
    LEDState = LED_OFF_STATE;
  }
  else
  {
    SET_LED();
    LEDState = LED_ON_STATE;
  }  
  P1DIR |= 0x11;                        //  P1.0,4 output
} 

void SystemInit()
{
  
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer

 BCSCTL1 = CALBC1_1MHZ;                // Set DCO
 DCOCTL = CALDCO_1MHZ;
}

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;          // Stop watchdog timer
  LEdInit(LED_OFF_STATE);           //LED state initialization
  SPISetup();                       //USi module in SPI mode initialization
  InitComm();                       //Communication initialization 
  
  while(1)
  {
    _BIS_SR(LPM0_bits + GIE);           // Enter LPM0 w/ interrupt
  
    switch (NextState)
    {
      case 0x00 :
        break;
      default :
        LEDState = NextState;          // new state
        break;
     }    
    if (LEDState == LED_OFF_STATE)
    {
      RESET_LED();
    }
    else
    {
      SET_LED();
    }
    USISRL = LEDState;            // prepares new communication with new state
    RESET_BUSY_FLAG();          // clears busy flag - ready for new communication
  }
}

#pragma vector=USI_VECTOR
__interrupt void USI_ISR(void)
{
  SET_BUSY_FLAG();            // set busy flag - slave is ready with a new communication
  NextState = USISRL;         // read new command
  USICNT = 8;                 // Load bit counter for next TX
  _BIC_SR_IRQ(LPM4_bits);     // Exit from LPM4 on RETI
}


































