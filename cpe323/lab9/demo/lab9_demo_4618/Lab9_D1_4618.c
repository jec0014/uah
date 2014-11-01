/*********************************************************************************/
//              Lab 5 - Serial Communication and SPI Interface           	 
//                                                                        	 
// Description: Using the MSP-EXP430FG4618 Development Tool establish a      	 
//           	data exchange between the MSP430FG4618 and MSP430F2013           
//           	devices using the SPI mode. The MSP430FG4618 uses the USCI       
//           	module while the MSP430F2013 uses the USI module.                
//              MSP430FG4618 communicates with PC via RS232 module using                                                          
//              USCI Serial Communication peripheral interface. This program     
//		takes user prompts the user to input a choice to turn ON or OFF  
//		the LED3 located on MSP430F2013. The user choice is communicated 
//		to MSP430FG4618 (master) via USCI serial interface and the 	 
//		corresponding action is communicated to MSP430F2013(slave) via 	 
//		SPI. Based on the user choice, MSP430F2013 will turn ON or OFF	 
//		the LED3. This is the master code that runs on MSP430FG4618.     
//                                                                        	 
//                                 Slave                    Master                  
//                              MSP430F2013              MSP430FG4618             
//                          -----------------          -----------------          
//                         |              XIN|-    /|\|              XIN|-        
//                         |                 |      | |                 |  32kHz xtal
//                         |             XOUT|-     --|RST          XOUT|-           
//                         |                 |        |                 |            
//                   LED <-|P1.0             |        |                 |            
//                         |         BF /P1.4|------->|P3.0/BF          |  
//                         |         SDI/P1.7|<-------|P3.1/UCB0SIMO    | 
//                         |         SDO/P1.6|------->|P3.2/UCB0SOMI    |  
//                         |        SCLK/P1.5|<-------|P3.3/UCB0CLK     | 
//                                                                        
// Instructions:                                                          
//                                                                        
// 1. Set the following parameters in hyperterminal                       
//						Port :		COM1      
//						Baud rate :	19200     
//						Data bits:	8         
//						Parity:		None      
//						Stop bits:	1         
//						Flow Control:	None	  
// 2. This lab requires to configure the USI module of MSP430F2013        
//    as slave and MSP430FG4618 as master in SPI mode.                    
// 3. Connect the following jumpers on header 1 (H1) on the experimenter's  
//    board.  [1-2], [3-4], [5-6], [7-8]                                  
/*		   H1	
		_______
	      1|-------|2
	      3|-------|4
	      5|-------|6
	      7|-------|8		
	       |_______|

      Jumper must be present on	PWR1, PWR2 and JP2. 

// 4. Download and run this code by the connecting the FET debugger to 
//    JTAG1 on the experimenter's board.
// 5. Make sure the device selected is MSP430FG4618 in the General Options of IAR.

**************************************************************************/

#include  "msp430xG46x.h"
#include <stdio.h>

#define LED_ON_STATE    0x31	// character '1'
#define LED_OFF_STATE   0x30    // character '0'
#define LED_NUL_STATE   0x00    // character NULL - used for dummy write operation
                
#define LED_ON          0x01
#define LED_OFF         0x00

unsigned char ch;  // hold char from UART RX
unsigned char rx_flag;   // receiver rx status flag

char gm1[] = "Press 'y' to turn ON and 'n' to turn OFF the LED";
char gm2[] = "Type in 'y' or 'n'!";

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
  UCA0BR0=54; 			// baud rate - 1048576 Hz / 19200
  UCA0BR1=0; 			//
  UCA0MCTL=0x0A; 		// Modulation
  UCA0CTL1 &= ~BIT0; 		// Software reset
  IE2 |=UCA0RXIE;               // Enable USCI_A0 RX interrupt
}

void main(void)
{
  
  WDTCTL = WDTPW+WDTHOLD;	// Stop watchdog timer

  Serial_Initialize();
  SPISetup();
  
  _EINT();				  // Enable global interrupts
  
  for (int z = 100; z > 0; z--);        // Delay to allow baud rate stabilize
  
  // Greeting Message
  for(int i = 0; i < 49; i++) {
        ch = gm1[i];
        UART0_putchar(ch); // print the greeting message on hyperterminal
    }
  UART0_putchar('\n');    // newline
  UART0_putchar('\r');    // carriage return
  
  while(1)
  {
  
    while(!(rx_flag&0x01)); // wait until receive the character from HyperTerminal
    rx_flag = 0;            // clear rx_flag
    
    switch (ch)
    {
    case  'y' :
              SPISetState(LED_ON_STATE);
              for(int i = 1000; i > 0;i--); //delay
              UART0_putchar(SPIGetState());  // prints the current state of LED 
              				      // '1' - ON ; '0' - OFF
          break;
    case  'n'  :
              SPISetState(LED_OFF_STATE);
              for(int i = 1000; i > 0;i--); //delay
              UART0_putchar(SPIGetState()); // prints the current state of LED
					    // '1' - ON ; '0' - OFF
          break;
    default     :
                for(int i = 0; i < 20; i++) {
                    ch = gm2[i];
                    UART0_putchar(ch); // print the greeting message on hyperterminal
                }
                UART0_putchar('\n');    // newline
                UART0_putchar('\r');    // carriage return
          break;
    }
  }
}

// Interrupt for USCI Rx 
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIB0RX_ISR (void)
{
  ch = UCA0RXBUF;  // character received is moved to a variable
  rx_flag=0x01;          // signal main function receiving a char
}
