////////////////////////////////////////////////////////////////////////////
//Program: Lab 10 demo 2
//Description: This code interfaces with an imaginary thumbstick that has
//    x and y axis output from 0 to 3 v.  The data is sent as a percentage
//    value to the UAH serial app
//By: Micah Harvey
//Date: July 10, 2013
//Baud rate: 38400
////////////////////////////////////////////////////////////////////////////

#include <msp430xG46x.h>

void turn_off();
void sendData();
void UART_putchar(char);
void initializeUART();

volatile long int ADCXval, ADCYval;
volatile float Xper, Yper;

void main(void)
{
  WDTCTL = WDTPW +WDTHOLD;              // Stop WDT
  
  initializeUART();	                // Setup UART for RS-232
  
  P6DIR &= ~0x05;	// Configure P6.3 and P6.7 as input pins  
  P6SEL |= 0x05;	// Configure P6.3 and P6.7 as analog pins

  //Set up timer to send ADC info to PC overy 100 ms
  TACCR0 = 3277;                //3277 / 32768 Hz = 0.1s
  TACTL = TASSEL_1 + MC_1;      //ACLK, up mode  
  TACCTL0 = CCIE;               //enabled interupt

  //Set up ADC 12
  ADC12CTL0 = ADC12ON + SHT0_6 + MSC; // configure ADC converter
  ADC12CTL1 = SHP + CONSEQ_1;     // Use sample timer, single sequence
  ADC12MCTL0 = INCH_0;            // ADC chan 0 is A3 pin - Stick X-axis
  ADC12MCTL1 = INCH_5 + EOS;      // ADC chan 1 is A7 pin - Stick Y-axis
				  //EOS - End of Sequence for Conversions	
  ADC12IE |= 0x02;                           // Enable ADC12IFG.8
  for (int i = 0; i < 0x3600; i++);         // Delay for reference start-up
  ADC12CTL0 |= ENC;                         // Enable conversions
  
  _EINT();
  
  while (1)
  {
    ADC12CTL0 |= ADC12SC;                   // Start conversions 
    __bis_SR_register(LPM0_bits + GIE);	    // enter LPM0
  }

}

///////////////////////////////////////////////////////////////////////////////
// User-defined function definitions
///////////////////////////////////////////////////////////////////////////////

void UART_putchar(char c)
{
    while(!(IFG2 & UCA0TXIFG)); //Wait for previous character to be sent
    UCA0TXBUF = c;              //Send byte to the buffer for transmitting
}

//Initializes UART control registers for transmitting data, as well as baud rate
void initializeUART(void)  
{
  
  P2SEL |= BIT4 + BIT5;          //Set up Rx and Tx bits
  UCA0CTL0 = 0;                  //Set up default RS-232 protocol         
  UCA0CTL1 |= BIT0 + UCSSEL_2;   //Disable device, set clock
  UCA0BR0 = 27;                  //1048576 Hz / 38400 = 54.5
  UCA0BR1 = 0;
  UCA0MCTL = 0x94;
  UCA0CTL1 &= ~BIT0;             //Start UART device
}

void sendData(void)
{
  Xper = (ADCXval*3/4095*100/3);        //calculate percentage outputs
  Yper = (ADCYval*3/4095*100/3);
  
  int i;
  
  //Send packet via rs-232
  UART_putchar(0x55);                   //send header

  //Use character pointers to send one byte of float X and Y value at a time
  char *xpointer=(char *)&Xper;
  char *ypointer=(char *)&Yper;
  
  //Send x percentage float one byte at a time
  for(i=0; i<4; i++)
  {
    UART_putchar(xpointer[i]);
  }
  
  //Send y percentage float one byte at a time
  for(i=0; i<4; i++)
  {
    UART_putchar(ypointer[i]);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Interrupt service routines
///////////////////////////////////////////////////////////////////////////////

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR(void)
{
  ADCXval = ADC12MEM0;                    // Move results, IFG is cleared
  ADCYval = ADC12MEM1;
  __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
}

#pragma vector = TIMERA0_VECTOR
__interrupt void timerA_isr()
{
  sendData();
  __bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0
}



































