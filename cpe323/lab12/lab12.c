/***********************************************************************
 * Title: LAB 12
 * Date Due: Nov 2x
 * 
 * Description:
 * 		This program does stuff
 *
 * By: Christopher 'ctag' Bero <csb0019@uah.edu>
 * Project: https://github.com/ctag/uah
 *
 * This software is licensed under the GPLv3,
 * consult the LICENSE file for more information.
 **********************************************************************/

/*
 * Include statements
 */
#include <msp430.h> // The *correct* include statement
#include <string.h> // For manipulating char[]?, doesn't seem to work with mspgcc :-/
#include <stdio.h>	// For printf functions
#include "gateway.h" // For wireless functions

/*
 * Global variables
 */
#define SW1 (0x01&P1IN)	// B1 - P1.0 switch SW1 
#define SW2 (0x02&P1IN)	// B2 - P1.1 switch SW2

#define SERVER_ADDRESS 0x1
#define CLIENT_ADDRESS 0x2

#define LED2_TOGGLE_COMMAND 0x02

extern char paTable[];
extern char paTableLen;

char led2TogglePacket[3];
char timePacket[9];
char rxBuffer[256];
char len;

volatile float millisec = 0;

/*
 * Function Definitions
 */

void Delay_Debounce(void)
{
	//for(int i=0;i<2500;i++);         // POR software delay
	
	// Constant delay debounce
    int factor = (SCFQCTL / 30);
	int looper = (20 * factor);
	for (int c = 0; c < looper; c++)
	{ asm("NOP"); }
}

/*
 * Function to initialize ports for interfacing TI CC1100 (TI_CC) 
 */
void  TI_CC_Initialize()
{
	TI_CC_LED_PxDIR |= TI_CC_LED1;           // Port 1 LED set to Outputs
	TI_CC_LED_PxOUT |= TI_CC_LED1;           // Initialize, turn on LED1
        TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;
	TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;    // Initialize, clear 
                                            // TI_CC_GDO0_PIN bit which is
                                            //set after data packet transfer          
	TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;      // Enable interrupt to occur at
                                            // the end of a data packet
}	// end TI_CC_Initialize (Transmitter version)

/*
 * Function to support MSP430 transmitter initialization to support wireless 
 * communication  via SPI provided by the TI CC1100 chip which attaches
 * to the transmitting TI MSP430 experimenter board.
 */
void Transmitter_Initialize()
{  
        Delay_Debounce();
	TI_CC_SPISetup();                       // Initialize SPI port
	TI_CC_PowerupResetCCxxxx();             // Reset CCxxxx
	writeRFSettings();                      // Write RF settings to config reg
	TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen); 
                                            // Write PATABLE
	TI_CC_Initialize();                     // Initialize CCxxx on port 1.
	TI_CC_SPIStrobe(TI_CCxxx0_SRX);         // Initialize CCxxxx in RX mode.
}	// end Transmitter_Initialize

/*
 * USART in UART Mode 
 */
void UART_Initialize(void) 
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

/*
 * Write a char to the serial buffer
 */
void UART_Write(char c) {
	while (!(IFG2 & UCA0TXIFG));
	UCA0TXBUF = c;
} 

/*
 * Write a static debugging message to serial buffer
 */
void UART_Toggle_SW2 (void) {
	char message[8];
	sprintf(message, "Toggling"); // prints string to char
	for(int i=0;i<8;i++) // Send character by character   
	{     
		while (!(IFG2 & UCA0TXIFG));     
		UCA0TXBUF = message[i];   
	}
	UART_Write('\r');
	UART_Write('\n');
}

void main(void)
{
	WDTCTL = (WDTPW|WDTHOLD); // halt watchdog
	
	// Setup Timer_A, leave halted
	TACTL = TASSEL_2 + ID_3; // Select smclk/8 and up mode
	// To start: TACTL |= MC_1;
	TACCR0 = 131; // 1ms interval   
	TACCTL0 = CCIE; // Capture/compare interrupt enable 
	
	_EINT();	// Enable interrupts

	UART_Initialize(); // Configure serial UART registers
	
	P1IE |= 0x03;		// Enable P1 interrupt for bit 0 and 1
	P1IES |= 0x03;		// Set interrupt call to falling edge
	P1IFG &= ~(0x03);	// Clear interrupt flags
	P2DIR |= 0x02;		// Set P2.1 and P2.2 to output (0000_0110) 
	P2OUT = 0x02;		// Set P2OUT to 0000_0010b
	Transmitter_Initialize();      

	// Go ahead and set up the packet for toggleing LED2 on the server board
	// as it will not change.
	led2TogglePacket[PLENGTHINDEX] = PSIZE - 1;  
	led2TogglePacket[PADDRINDEX] = SERVER_ADDRESS;
	led2TogglePacket[PDATAINDEX] = LED2_TOGGLE_COMMAND;
	
	// Setup timePacket
	timePacket[0] = 5;
	timePacket[1] = 0x1;
	
	while (1)
	{
		//_BIS_SR(LPM3_bits + GIE);	// Enter LPM0, enable interrupts
		
	}
}

/*
 * Timer_A interrupt service routine
 */
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA_ISA(void)
{
	millisec++;
	UART_Toggle_SW2();
	P2OUT ^= 0x02;
	//_BIC_SR_IRQ(LPM0_bits); // Clear LPM0 bits from 0(SR)
}

/*
 * Port 1 interrupt service routine
 */
#pragma vector = PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
  Delay_Debounce();

	if (((SW1) == 0) && ((SW2) != 0)) // SW1 is pressed
	{
		millisec = 0;
		
		TACTL |= MC_1;
		
		while (SW1 == 0) {
			
		}
		
		TACTL &= ~MC_1;
		
		char * timePointer = (char *)&millisec;
		
		int i = 0;
		while (i < 9) {
			UART_Write(timePointer[i]);
			UART_Write('_');
			i++;
		}
		
		for (int c = 0; c < 7; c++) {
			timePacket[c+2] = timePointer[c];
		}
		
		RFSendPacket(timePacket, 9);
		
	} else if (((SW2) == 0) && ((SW1) != 0)) // SW2 is pressed
	{
		RFSendPacket(led2TogglePacket, PSIZE);
	}
	
	if(P1IFG & TI_CC_GDO0_PIN)              // Command received from RF RX active
     {   
        if( RFReceivePacket(rxBuffer,&len))   // check if packet is received       
	{
			
            if(rxBuffer[0] == CLIENT_ADDRESS) // Make sure this message is intended for the client
            {
          
              //if server sends LED2 toggle command, toggle LED2
              if(rxBuffer[1] == LED2_TOGGLE_COMMAND && len==2) //Messages longer than length 2 should be treated differently
              {
                  P2OUT ^= 0x02;
                  //UART_Toggle_SW2();
              } 
              else { // Send to UART
			  int i = 1;
			  while (rxBuffer[i] != '\0')
			  {
				  UART_Write(rxBuffer[i]);
				  i++;
				  if (i > len) {
					  break;
				  }
			  }
            }
              
            }
        }
	P1IFG &= ~(TI_CC_GDO0_PIN); // reset of TI_CC_GDO0_PIN bit which is set after data packet transfer  
	
    }

	P1IFG &= ~BIT1;		// Clear P1.1 IFG
	P1IFG &= ~BIT0;		// Clear P1.0 IFG
}









































