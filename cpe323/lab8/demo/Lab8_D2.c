//**********************************************************************************
// FG4618 Board Demo - Real-time clock in Hyperterminal
//
// Filename: rtClock_uart.c
//
// Description: This program maintains real-time clock
// and sends time (10 times a second) to the workstation
// through a serial asynchronous link (uart).
// This program displays the system clock in "sssss:tsec" format to HyperTerminal.
// Instructions: Set the following parameters in hyperterminal
// Port : COM1
// Baud rate : 19200
// Data bits: 8
// Parity: None
// Stop bits: 1
// Flow Control: None
//
// Authors: Max Avula (ma0004@uah.edu), A. Milenkovic, milenkovic@computer.org
// UAH - October 2010
// Built with IAR Embedded Workbench IDE Version: 4.20.1.30017
//********************************************************************************** 
#include <msp430xG46x.h>
#include <stdio.h>

// Current time variables
unsigned int sec=0; // seconds 
unsigned char tsec=0; // 1/10 second 
char Time[8]; // string to keep current time 

//Function Declarations 
void SetTime(void); 
void SendTime(void); 
////////////////////////////////////////////////////////////////// 
//UART0 Initialization 						  
////////////////////////////////////////////////////////////////// 
void UART_Initialize(void) // USART in UART Mode 
{   
	P2SEL |= BIT4+BIT5; // Set UC0TXD and UC0RXD to transmit and receive data   
	UCA0CTL1 |= BIT0; // Software reset   
	UCA0CTL0 = 0; // USCI_A0 control register   
	UCA0CTL1 |= UCSSEL_2; // Clock source SMCLK   
	UCA0BR0=54; // 1048576 Hz / 19200   
	UCA0BR1=0; //   
	UCA0MCTL=0x0A; // Modulation   
	UCA0CTL1 &= ~BIT0; // Undo software reset 
}  

// Sets the real-time clock variables 
void SetTime(void) {   
	tsec++;   
	if(tsec == 10)   
	{     
		tsec = 0;     
		sec++;     
		P5OUT^=BIT1;   
	} 
} 

// Sends the time to the HyperTerminal 
void SendTime(void) {   
	sprintf(Time, "%05d:%01d", sec, tsec); // prints time to a string   
	for(int i=0;i<8;i++) // Send character by character   
	{     
		while (!(IFG2 & UCA0TXIFG));     
		UCA0TXBUF = Time[i];   
	}   
	while (!(IFG2 & UCA0TXIFG));   
	UCA0TXBUF = 0x0D; // Carriage Return 
} 

//////////////////////////////////////////////////////////////// 
//                         MAIN					  // 
//////////////////////////////////////////////////////////////// 
void main(void) 
{   WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer   
	UART_Initialize(); //Initialize UART   /* Initialize Timer A to measure 1/10 sec */   
	TACTL = TASSEL_2 + MC_1+ ID_3; // Select smclk/8 and up mode   
	TACCR0 = 13107; // 100ms interval   
	TACCTL0 = CCIE; // Capture/compare interrupt enable 
	P5DIR |= 0x02;   
	while(1) // Infinite loop   
	{     
		_BIS_SR(LPM0_bits + GIE); // Enter LPM0 w/ interrupts     
		SendTime(); // Send Time to HyperTerminal   
	} 
} 

// Interrupt for the timer 
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA_ISA(void)
{
	SetTime(); // Set Clock
	_BIC_SR_IRQ(LPM0_bits); // Clear LPM0 bits from 0(SR)
}
