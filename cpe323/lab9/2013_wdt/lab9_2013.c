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

// Usage: WDTCTL = WDT_CONFIG;
#define WDT_CONFIG_250 (WDTPW|WDTCNTCL|WDTSSEL|WDTIS0)		// Set bits to give us 0.250s watchdog
#define WDT_CONFIG_1000 (WDTPW|WDTCNTCL|WDTSSEL)			// Set bits to give us 1s watchdog
#define WDT_INTERVAL_250 (WDTPW|WDTCNTCL|WDTTMSEL|WDTSSEL|WDTIS0) // Set bits to have 0.250s timer
#define WDT_INTERVAL_1000 (WDTPW|WDTCNTCL|WDTTMSEL|WDTSSEL) // Set bits to have 1s timer
#define WDT_HALT (WDTPW|WDTHOLD) // Set bits to halt the timer

#define LED_4X_STATE    0x34	// character '4' - 4X=4.0Hz
#define LED_3X_STATE    0x33	// character '3' - 3X=2.0Hz
#define LED_2X_STATE    0x32	// character '2' - 2X=1.0Hz
#define LED_1X_STATE    0x31	// character '1' - 1X=0.5Hz
#define LED_OFF_STATE   0x30    // character '0'
#define LED_NUL_STATE   0x00    // character NULL - dummy write

#define SET_LED()       P1OUT |= 0x01;
#define RESET_LED()     P1OUT &= ~0x01;

int LED_delays = 8;
int LED_counter = 0;
int LED_on = 0;

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
}

void main(void)
{
	//WDTCTL = WDT_ADLY_250;	// 1 s interval timer
	WDTCTL = (WDTPW|WDTHOLD);
	
	SPISetup();                       //USi module in SPI mode initialization
	InitComm();                       //Communication initialization 
	
	P1DIR |= BIT0; // LED3 as output
	P1OUT &= ~BIT0; // LED off
	//P1OUT |= BIT0;
    
	IE1 |= BIT0; // BIT0 is WDTIE
	_EINT();			// Enable interrupts
	// __enable_interrupt(); // Same as _EINT()?
	
	//IE1 |= WDTIE;                     // Enable WDT interrupt
	//_BIS_SR(LPM0_bits + GIE);         // Enter LPM0 w/ interrupt
	
	WDTCTL = WDT_INTERVAL_250;
	//WDTCTL = WDT_ADLY_250;
        
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
		  LED_delays = 8;
		}
		else if (LEDState == LED_2X_STATE)
		{
		  LED_delays = 4;
		} else if (LEDState == LED_3X_STATE)
		{
		  LED_delays = 2;
		} else if (LEDState == LED_4X_STATE)
		{
		  LED_delays = 1;
		} else
		{
		}
                
                //P1OUT ^= BIT0;
                
	}
}

/*
 * USI ISR
 */
#pragma vector = USI_VECTOR
__interrupt void USI_ISR(void)
{
  NextState = USISRL;         // read new command
  USICNT = 8;                 // Load bit counter for next TX
  _BIC_SR_IRQ(LPM4_bits);     // Exit from LPM4 on RETI
}

/*
 * Watchdog Timer interrupt service routine
 */
#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void)
{
  P1OUT ^= BIT0;
	if (LED_on == 0)
	{
		if (LED_counter >= LED_delays)
		{
			LED_counter = 0;
			P1OUT |= BIT0;
			LED_on = 1;
		} else {
			LED_counter++;
		}
	} else {
		P1OUT &= ~BIT0;
		LED_on = 0;
	}
}



























