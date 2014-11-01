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

// Set MOSI definitions
#define LED_ON_STATE    0x31	// character '1'
#define LED_OFF_STATE   0x30    // character '0'
#define LED_NUL_STATE   0x00    // character NULL - used for dummy write operation

// Set MISO definitions
#define LED_ON          0x01
#define LED_OFF         0x00

void SPISetup(void)
{
  UCB0CTL0 = UCMSB + UCMST + UCSYNC; 	// sync. mode, 3-pin SPI, Master mode, 8-bit data
  UCB0CTL1 = UCSSEL_2 + UCSWRST;   	// SMCLK and Software reset                  
  UCB0BR0 = 0x02; 			// Data rate = SMCLK/2 ~= 500kHz                          
  UCB0BR1 = 0x00;
  P3SEL |= BIT1 + BIT2 + BIT3;		// P3.1,P3.2,P3.3 option select               
  UCB0CTL1 &= ~UCSWRST; 		// **Initialize USCI state machine**
}

unsigned char SPIGetState(void)
{
    while((P3IN & 0x01));             // Verifies busy flag
    IFG2 &= ~UCB0RXIFG;    
    UCB0TXBUF = LED_NUL_STATE;        // Dummy write to start SPI
    while (!(IFG2 & UCB0RXIFG));      // USCI_B0 TX buffer ready?
    return UCB0RXBUF;                  
}

void SPISetState(unsigned char State)
{
   while(P3IN & 0x01);                // Verifies busy flag
   IFG2 &= ~UCB0RXIFG;  
   UCB0TXBUF = State;                 //  write new state
   while (!(IFG2 & UCB0RXIFG));       // USCI_B0 TX buffer ready?
}


void main(void)
{
	WDTCTL = (WDTPW|WDTHOLD);
	
	Serial_Initialize();
	SPISetup();
	
	_EINT();	// Enable interrupts
	
	IE1 |= WDTIE;                     // Enable WDT interrupt
	//_BIS_SR(LPM0_bits + GIE);         // Enter LPM0 w/ interrupt
	
	WDTCTL = WDT_ADLY_1000;
	
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
	char currentState = SPIGetState();
	if (currentState == '0')
	{
		SPISetState(LED_ON_STATE);
	} else {
		SPISetState(LED_OFF_STATE);
	}
}





























