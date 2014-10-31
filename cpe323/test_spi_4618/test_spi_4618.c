/***********************************************************************
 * Title: Test - SPI for MSP4618
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

#define SW1 0x01&P1IN	// B1 - P1.0 switch SW1 
#define SW2 0x02&P1IN	// B2 - P1.1 switch SW2

void main(void)
{
	//WDTCTL = WDT_ADLY_250;	// 1 s interval timer
	//WDTCTL = (WDTPW|WDTHOLD);
	
	//_EINT();			// Enable interrupts
	// __enable_interrupt(); // Same as _EINT()?
	
	//IE1 |= WDTIE;                     // Enable WDT interrupt
	//_BIS_SR(LPM0_bits + GIE);         // Enter LPM0 w/ interrupt

	while (1)
	{
		asm("NOP");
	}
}

/*
 * Watchdog Timer interrupt service routine
 */
#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void)
{

}

/*
 * Port 1 interrupt service routine
 */
#pragma vector = PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
	// Constant delay debounce
	int factor = (SCFQCTL / 30);
	int looper = (20 * factor);
	for (int c = 0; c < looper; c++)
	{ asm("NOP"); }

	if (((SW1) == 0) && ((SW2) != 0)) // SW1 is pressed
	{
	
	} else if (((SW2) == 0) && ((SW1) != 0)) // SW2 is pressed
	{
	
	}

	P1IFG &= ~BIT1;		// Clear P1.1 IFG
	P1IFG &= ~BIT0;		// Clear P1.0 IFG
}





























