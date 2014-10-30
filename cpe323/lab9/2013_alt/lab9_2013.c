/***********************************************************************
 * Title: LAB 9 - MSP~2013
 * Date Due: 30-10-14
 * 
 * Description:
 * Listens on SPI and adjusts the rate of a blinking LED.
 *
 * By: Christopher 'ctag' Bero <csb0019@uah.edu>
 * Project: https://github.com/ctag/uah
 *
 * This software is licensed under the GPLv3,
 * consult the LICENSE file for more information.
 **********************************************************************/

#include <msp430.h> // The *correct* include statement

#define LED_4X_STATE    0x34	// character '4' - 4X=4.0Hz
#define LED_3X_STATE    0x33	// character '3' - 3X=2.0Hz
#define LED_2X_STATE    0x32	// character '2' - 2X=1.0Hz
#define LED_1X_STATE    0x31	// character '1' - 1X=0.5Hz
#define LED_OFF_STATE   0x30    // character '0'
#define LED_NUL_STATE   0x00    // character NULL - dummy write

#define SET_LED()       P1OUT |= 0x01;
#define RESET_LED()     P1OUT &= ~0x01;

#define SET_BUSY_FLAG()   P1OUT |= 0x10;
#define RESET_BUSY_FLAG() P1OUT &= ~0x10;

unsigned char LEDState;
unsigned char NextState;

long unsigned int delay_1x = 90000;
long unsigned int delay_2x = 45000;
long unsigned int delay_3x = 30000;
long unsigned int delay_4x = 22500;
long unsigned int delay = 90000;

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
 USISRL = 0x00;          // set LED state
 RESET_BUSY_FLAG();          // reset busy flag
}

void main(void)
{
	//WDTCTL = WDT_ADLY_250;	// 1 s interval timer
	WDTCTL = (WDTPW|WDTHOLD);
	
	 BCSCTL1 = CALBC1_1MHZ;                // Set DCO
 DCOCTL = CALDCO_1MHZ;
	
	SPISetup();                       //USi module in SPI mode initialization
	InitComm();                       //Communication initialization 
	
	P1DIR |= BIT0; // LED3 as output
	P1OUT &= ~BIT0; // LED off
	//P1OUT |= BIT0;
	
	_EINT();			// Enable interrupts
	// __enable_interrupt(); // Same as _EINT()?
	
        
	while (1)
	{
		//_BIS_SR(LPM0_bits + GIE);           // Enter LPM0 w/ interrupt
                
          
		switch (NextState)
		{
		  case 0x00 :
			break;
		  default :
			LEDState = NextState;          // new state
			break;
		}
		 
		if (LEDState == LED_1X_STATE)
		{
		  delay = delay_1x;
		}
		else if (LEDState == LED_2X_STATE)
		{
		  delay = delay_2x;
		} else if (LEDState == LED_3X_STATE)
		{
		  delay = delay_3x;
		} else if (LEDState == LED_4X_STATE)
		{
		  delay = delay_4x;
		} else
		{
		}
		
		for (long unsigned int c = 0; c < delay; c++)
		{
			asm("NOP");
                        //P1OUT ^= BIT0;
		}
		
		P1OUT ^= BIT0;
		
		USISRL = 0x00;            // prepares new communication with new state
    RESET_BUSY_FLAG();          // clears busy flag - ready for new communication
	}
}

/*
 * USI ISR
 */
#pragma vector = USI_VECTOR
__interrupt void USI_ISR(void)
{
SET_BUSY_FLAG();            // set busy flag - slave is ready with a new communication
  NextState = USISRL;         // read new command
  USICNT = 8;                 // Load bit counter for next TX
  //_BIC_SR_IRQ(LPM4_bits);     // Exit from LPM4 on RETI
}



























