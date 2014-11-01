/***********************************************************************
 * Title: Test - SPI for MSP430x2013
 * Date Due: NEVA!
 * 
 * Description:
 *
 * By: Christopher 'ctag' Bero <csb0019@uah.edu>
 * Project: https://github.com/ctag/uah
 *
 * This software is licensed under the GPLv3,
 * consult the LICENSE file for more information.
 **********************************************************************/

#include <msp430.h> // The *correct* include statement

// Set MOSI definitions
#define LED_ON_STATE    0x31	// character '1'
#define LED_OFF_STATE   0x30    // character '0'
#define LED_NUL_STATE   0x00    // character NULL - used for dummy write operation

// Set MISO definitions
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

void main(void)
{
	//WDTCTL = WDT_ADLY_250;	// 1 s interval timer
	WDTCTL = (WDTPW|WDTHOLD);
	
	LEdInit(LED_OFF_STATE);           //LED state initialization
	SPISetup();                       //USi module in SPI mode initialization
	InitComm();                       //Communication initialization 
	
	//_EINT();			// Enable interrupts
	// __enable_interrupt(); // Same as _EINT()?
	
	//IE1 |= WDTIE;                     // Enable WDT interrupt
	//_BIS_SR(LPM0_bits + GIE);         // Enter LPM0 w/ interrupt

	while (1)
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





























