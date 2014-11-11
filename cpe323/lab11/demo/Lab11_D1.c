//******************************************************************************
//   MSP430xG461x Demo - DAC12_0, Output Voltage SINEWAVE on DAC0
//   Device: TI Experimeter's Board
//
//   Description: Using DAC12_0 and 2.5V ADC12REF reference with a gain of 1,
//   output sinisoidal wave on P6.6, y=1.25(1+sin(x)).
//   Normal mode is LPM0 with CPU off.  WDT used
//   to provide ~0.064ms interrupt used to wake up the CPU and update the DAC
//   with software. Use internal 2.5V Vref.
//   ACLK = 32kHz, SMCLK = MCLK = WDTCLK = default DCO 1048576Hz
//
//
//                MSP430xG461x
//             -----------------
//         /|\|              XIN|-
//          | |                 | 32kHz
//          --|RST          XOUT|-
//            |                 |
//            |        DAC0/P6.6|--> sine (10Hz)
//            |                 |
//
// Authors: Aleksandar Milenkovic, milenkovic@computer.org
//          Max Avula, ma0004@uah.edu
//			UAHuntsville
// Date:  March 2012
// Built with IAR Embedded Workbench IDE Version: 6.3.11.2079
//******************************************************************************
#include "msp430xG46x.h"
#include "sine_lut_256.h"  /*256 samples are stored in this table */

void main(void) {
  
  unsigned int i;

  WDTCTL = WDTPW + WDTHOLD;                   // Stop WDT
  ADC12CTL0 = REF2_5V + REFON;                // Internal 2.5V ref on
  for (i=50000; i>0; i--);                    // Delay to allow Ref to settle
  __disable_interrupt();                      // Disable Interrupts
  DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;   //Sets DAC12
  CCTL0 = CCIE;                                // CCR0 interrupt enabled
  CCR0 = 410;                                   // Sets Timer Freq (1048576*0.1sec/256)
  TACTL = TASSEL_2 + MC_1;                     // set clock to SMCLK, up mode
  i=0;
  
  while (1)
  {
    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0, interrupts enabled
    DAC12_0DAT = LUT256[i];
    DAC12_0DAT &= 0xFFF;
    i=(i+1)%256;
  }
}

#pragma vector = TIMERA0_VECTOR
__interrupt void TA0_ISR(void)
{
  __bic_SR_register_on_exit(LPM0_bits);     // Exit LPMx, interrupts enabled
}
