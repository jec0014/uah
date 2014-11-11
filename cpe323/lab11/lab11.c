/***********************************************************************
 * Title:
 * Date Due:
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
#include "waveform.h" // Local header

#define SW1 0x01&P1IN	// B1 - P1.0 switch SW1 
#define SW2 0x02&P1IN	// B2 - P1.1 switch SW2

unsigned short int index = 0; // waveform array index
unsigned short int array = 0; // Alternate waveforms
unsigned short int amp = 1; // Alternate amplitude

void main(void)
{
	WDTCTL = (WDTPW|WDTHOLD);
	
	// Setup ADC
	ADC12CTL0 = REF2_5V + REFON;	// internal 2.5Vref
	for (i=50000; i>0; i--);		// Setup delay for Vref
	__disable_interrupt();			// I don't feel this is necessary, but whatever
	DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;	//Setup DAC12
	CCTL0 = CCIE;	// CapCom interrupt enabled
	CCR0 = 273;		// Setup CCR0 delay [(1/15)/256*1048576]
	TACTL = TASSEL_2 + MC_1;	// TimerA clock set to smclk, up mode
	
	// Setup P1 input interrupt
	P1IE |= 0x0003;		// P1.0 interrupt enabled
	P1IES |= 0x0003;	// P1.0 hi -> low edge
	P1IFG &= ~0x0003;	// Clear P1.0 IFG
	
	//_EINT();	// Enable interrupts
	
	//IE1 |= WDTIE;                     // Enable WDT interrupt
	//_BIS_SR(LPM0_bits + GIE);         // Enter LPM0 w/ interrupt

	while (1)
	{
		__bis_SR_register(LPM0_bits + GIE);		// Enter LPM0, interrupts enabled
		switch (array) {
			case 0:
				DAC12_0DAT = (sine[index]/amp);		// Load next 1/256th chunk of the waveform
			break;
			case 1:
				DAC12_0DAT = (triangle[index]/amp);		// Load next 1/256th chunk of the waveform
			break;
			case 2:
				DAC12_0DAT = (saw[index]/amp);		// Load next 1/256th chunk of the waveform
			break;
		}
		
		DAC12_0DAT &= 0xFFF;			// Masking bits
		index=(index+1)%256;			// queue up next index
	}
}

/*
 * TimerA interrupt service routine
 */
#pragma vector = TIMERA0_VECTOR
__interrupt void TA0_ISR(void)
{
  __bic_SR_register_on_exit(LPM0_bits);     // Exit LPMx, interrupts unchanged
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
		array++;
		if (array > 2) {
			array = 0;
		}
		index = 0;
	} else if (((SW2) == 0) && ((SW1) != 0)) // SW2 is pressed
	{
		switch (amp) {
			case 1:
				amp = 2;
			break;
			case 2:
				amp = 4;
			break;
			case 4:
				amp = 1;
			break;
			default:
				amp = 1;
			break;
	}

	P1IFG &= ~BIT1;		// Clear P1.1 IFG
	P1IFG &= ~BIT0;		// Clear P1.0 IFG
}





























