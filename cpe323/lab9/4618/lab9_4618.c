/***********************************************************************
 * Title: LAB 9 - MSP~4618
 * Date Due: 30-10-14
 * 
 * Description:
 * Reads input over serial and instructs MSP2013 over SPI to adjust
 * an LED's blink rate.
 *	
 
 Make sure:
 BAUD - 19200
 8N1
 Flow Control - OFF
 
 * By: Christopher 'ctag' Bero <csb0019@uah.edu>
 * Project: https://github.com/ctag/uah
 *
 * This software is licensed under the GPLv3,
 * consult the LICENSE file for more information.
 **********************************************************************/

#include <msp430.h> // The *correct* include statement
#include <stdio.h> // '(dot) EWW' (see what I did there?)

#define SW1 0x01&P1IN	// B1 - P1.0 switch SW1
#define SW2 0x02&P1IN	// B2 - P1.1 switch SW2

#define LED_4X_STATE    0x34	// character '4'
#define LED_3X_STATE    0x33	// character '3'
#define LED_2X_STATE    0x32	// character '2'
#define LED_1X_STATE    0x31	// character '1'
#define LED_OFF_STATE   0x30    // character '0'
#define LED_NUL_STATE   0x00    // character NULL - dummy write

unsigned char ch = '0';  // hold char from UART RX
unsigned char rx_flag = 0;   // receiver rx status flag

char serialMessage[] = "Enter 1, 2, 3, or 4: ";
char serialChastise[] = "Only 1-4 is allowed as valid input.";

void SPISetup(void)
{
  UCB0CTL0 = UCMSB + UCMST + UCSYNC; 	// sync. mode, 3-pin SPI, Master mode, 8-bit data
  UCB0CTL1 = UCSSEL_2 + UCSWRST;   	// SMCLK and Software reset                  
  UCB0BR0 = 0x02; 			// Data rate = SMCLK/2 ~= 500kHz                          
  UCB0BR1 = 0x00;
  P3SEL |= BIT1 + BIT2 + BIT3;		// P3.1,P3.2,P3.3 option select               
  UCB0CTL1 &= ~UCSWRST; 		// **Initialize USCI state machine**
}

/*
unsigned char SPIGetState(void)
{
    while((P3IN & 0x01));             // Verifies busy flag
    IFG2 &= ~UCB0RXIFG;    
    UCB0TXBUF = LED_NUL_STATE;        // Dummy write to start SPI
    while (!(IFG2 & UCB0RXIFG));      // USCI_B0 TX buffer ready?
    return UCB0RXBUF;                  
}
*/

void SPISetState(unsigned char State)
{
   while(P3IN & 0x01);                // Verifies busy flag
   IFG2 &= ~UCB0RXIFG;  
   UCB0TXBUF = State;                 //  write new state
   while (!(IFG2 & UCB0RXIFG));       // USCI_B0 TX buffer ready?
}

void UART0_putchar(char c) {
  // wait for other character to transmit
  while (!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = c;
}

void Serial_Initialize(void)
{
  P2SEL |= BIT4+BIT5; 		// Set UC0TXD and UC0RXD to transmit and receive data
  UCA0CTL1 |= BIT0; 		// Software reset
  UCA0CTL0 = 0; 		// USCI_A0 control register
  UCA0CTL1 |= UCSSEL_2; 	// Clock source SMCLK - 1048576 Hz
  UCA0BR0 = 54; 			// baud rate - 1048576 Hz / 19200
  UCA0BR1 = 0; 			//
  UCA0MCTL = 0x0A; 		// Modulation
  UCA0CTL1 &= ~BIT0; 		// Software reset
  IE2 |= UCA0RXIE;               // Enable USCI_A0 RX interrupt
}

void main(void)
{
	//WDTCTL = WDT_ADLY_250;	// 1 s interval timer
	WDTCTL = (WDTPW|WDTHOLD);
	
	Serial_Initialize();
	//SPISetup();
	
	_EINT();			// Enable interrupts
	// __enable_interrupt(); // Same as _EINT()?
	
	//IE1 |= WDTIE;                     // Enable WDT interrupt
	//_BIS_SR(LPM0_bits + GIE);         // Enter LPM0 w/ interrupt
	
	for (int z = 100; z > 0; z--);        // Delay to allow baud rate stabilize
	
	UART0_putchar('\n');    // newline
	UART0_putchar('\r');    // carriage return
	
	while (1)
	{
	
		for(int i = 0; i < 21; i++) {
			ch = serialMessage[i];
			UART0_putchar(ch); // print the greeting message on hyperterminal
		}
		
		while(rx_flag != 1); // wait until receive the character from HyperTerminal
		rx_flag = 0;            // clear rx_flag
		
		UART0_putchar('\n');    // newline
		UART0_putchar('\r');    // carriage return
		switch (ch)
		{
			case '1' :
				SPISetState(LED_1X_STATE);
			break;
			
			case '2' :
				SPISetState(LED_2X_STATE);
			break;
			
			case '3' :
				SPISetState(LED_3X_STATE);
			break;
			
			case '4' :
				SPISetState(LED_4X_STATE);
			break;
			
			default:
				for(int i = 0; i < 35; i++) {
					ch = serialChastise[i];
					UART0_putchar(ch); // print the greeting message on hyperterminal
				}
				UART0_putchar('\n');    // newline
				UART0_putchar('\r');    // carriage return
				UART0_putchar('\n');    // newline
				UART0_putchar('\r');    // carriage return
			break;
		}
		
		for(int i = 1000; i > 0;i--); //delay
		
	}
}

/*
 * Interrupt SR for USCI Rx 
 */
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIB0RX_ISR (void)
{
  ch = UCA0RXBUF;  // character received is moved to a variable
  rx_flag = 1;          // signal main function receiving a char
}

/*
 * Watchdog Timer interrupt service routine
 */
 /*
#pragma vector = WDT_VECTOR
__interrupt void watchdog_timer(void)
{

}
*/

/*
 * Port 1 interrupt service routine
 */
/*
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
*/







































