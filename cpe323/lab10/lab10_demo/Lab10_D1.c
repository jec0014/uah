/**********************************************************************************
//  TI Experimenter's Board Demo - Temperature Display in Hyperterminal
//
//  Filename: Temp_Display.c
//
//  Description: 
//              This program samples the on-chip temperature sensor, converts the sampled 
//              voltage from the sensor to temperature in degrees Celsius and 
//              Fahrenheit, and sends the temperature sensor through a RS232 link 
//              to the Hyper-terminal application.
//  Instructions: Set the following parameters in hyperterminal 
//						Port :		COM1
//						Baud rate :	38400
//						Data bits:	8
//						Parity:		None
//						Stop bits:	1
//						Flow Control:	None	
//
//  Authors: A. Milenkovic, milenkovic@computer.org
//			 Max Avula (ma0004@uah.edu)
//  	     UAH
//  March 2012
//  Built with IAR Embedded Workbench IDE Version: 6.3.11.2079
**********************************************************************************/

#include  <msp430xG46x.h>
#include  <stdio.h>

unsigned char thr_char;  /* hold char from UART RX*/
unsigned char rx_flag;   /* receiver rx status flag */

char gm1[] = "Hello! I am an MSP430. Would you like to know my temperature? (Y|N)";
char gm2[] = "Bye, bye!";
char gm3[] = "Type in Y or N!";

long int temp;
long int IntDegF;
long int IntDegC;

char NewKey[25];


// UART Initializaion
void UART_Initialize(void);
//send char function
void UART_putchar(char c);

void main(void)
{
  int i = 0;

  WDTCTL = WDTPW + WDTHOLD;   // Stop watchdog timer

  UART_Initialize();

  ADC12CTL0 = SHT0_8 + REFON + ADC12ON;
  ADC12CTL1 = SHP;                      // enable sample timer
  ADC12MCTL0 = 0x01A;
  ADC12IE = 0x001;

  rx_flag = 0;            // rx default state "empty"
  _EINT();                // enable global interrupts


Wait:
	//send a greeting message
	for(i = 0; i < 67; i++)
	{
		thr_char = gm1[i];
		UART_putchar(thr_char);
	}
    while(!(rx_flag&0x01)); // wait until receive the character from HyperTerminal
    rx_flag = 0;            // clear rx_flag
    UART_putchar(thr_char);
    UART_putchar('\n');    // newline
    UART_putchar('\r');    // carriage return
    
	// character input validation
    if ((thr_char != 'y') && (thr_char != 'n') && (thr_char !='Y') && (thr_char !='N')) 
	{
		for(i = 0; i < 15; i++) 
		{
			thr_char = gm3[i];
			UART_putchar(thr_char);
		}
		UART_putchar('\n');    // newline
		UART_putchar('\r');    // carriage return
		goto Wait;
    }

   if ((thr_char == 'y') || (thr_char == 'Y')) 
   {
       ADC12CTL0 |= ENC + ADC12SC;         // Sampling and conversion start
      _BIS_SR(CPUOFF + GIE);              // LPM0 with interrupts enabled

      //  oF = ((x/4096)*1500mV)-923mV)*1/1.97mV = x*761/4096 - 468
      //  IntDegF = (ADC12MEM0 - 2519)* 761/4096

      IntDegF = (temp - 2519) * 761;
      IntDegF = IntDegF / 4096;

      //  oC = ((x/4096)*1500mV)-986mV)*1/3.55mV = x*423/4096 - 278
      //  IntDegC = (ADC12MEM0 - 2692)* 423/4096

      IntDegC = (temp - 2692) * 423;
      IntDegC = IntDegC / 4096;

      //printing the temperature on hyperterminal
	  sprintf(NewKey, "T(F)=%ld\tT(C)=%ld\n", IntDegF, IntDegC);
	  for(i = 0; i < 25; i++) {
         thr_char = NewKey[i];
         UART_putchar(thr_char);
      }
      UART_putchar('\n');    // newline
      UART_putchar('\r');    // carriage return
      goto Wait;
   }

    if ((thr_char == 'n') || (thr_char == 'N')) 
	{
		for(i = 0; i < 9; i++) 
		{
			thr_char = gm2[i];
			UART_putchar(thr_char);
		}
		UART_putchar('\n');    // newline
		UART_putchar('\r');    // carriage return
   }
}

void UART_Initialize(void)
{

  P2SEL |= BIT4+BIT5;	// Set UC0TXD and UC0RXD to transmit and receive data	
  UCA0CTL1 |= BIT0;		// software reset
  UCA0CTL0 = 0;			// USCI_A0 control register
  UCA0CTL1 |= UCSSEL_2; // clock source SMCLK
  UCA0BR0=27;			// 1 MHz 38400
  UCA0BR1=0;			// 1 MHz 38400
  UCA0MCTL=0x94;		// Modulation
  UCA0CTL1 &= ~BIT0;	// software reset
  IE2 |=UCA0RXIE;       // Enable USCI_A0 RX interrupt
}


void UART_putchar(char c) {
  // wait for other character to transmit
  while (!(IFG2 & UCA0TXIFG));
  UCA0TXBUF = c;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCIA0RX_ISR (void)
{
  thr_char = UCA0RXBUF;
  rx_flag=0x01;          // signal main function receiving a char
  LPM0_EXIT;
}


#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
    temp = ADC12MEM0;                   // Move results, IFG is cleared
    _BIC_SR_IRQ(CPUOFF);                // Clear CPUOFF bit from 0(SR)
}