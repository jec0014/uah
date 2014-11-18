/* *********************************************************************************
// Description: Program samples the MSP430's analog temperature sensor (20 sps)
//              calculates the temperature in degrees Fahrenheit and Celsius, and
//              sends the calculated temperature wirelessly to a receiving node
//              using CC1101 RF transceiver. Receiveing node extracts the packet
//              and sends the data to UAH Serial App. 
// Platform:    TI Experimenter's board with MSP430FG4618/F2013 with CC1101
//
// Files:       BaseImplementation_re.c
//              Gateway.c  - CC1101 hardware abstraction layer
//              Gateway.h  - CC1101 header file
//             
// Setup:       2 TI Experimenter boards (transmitter and receiver nodes).
//              This program runs on the Receiver node.
//              The receiver node receives the wireless messages,
//              extracts the messages, and sends data over RS232 to 
//              a workstation that plots the temperature in degrees
//              Fahrenheit and Celsius in UAHSerialApp.
//
// Authors:     CC1101 hardware abstraction layer provided by TI;
//              Prepared by: Sunny Patel, Sjohn Chambers & Pam Mazurkivich
//              Edited and verifed by: Aleksandar Milenkovic
// Contact:     milenkovic@computer.org
// Date:        March 2013
// Notes:       Notes:
//              1.  Not configured for LOW POWER, 
//                  remove BATT jumper to conserve power when not in use.
//              2.  If the TI MSP430 devices are placed too close, 
//                  interference may cause data loss.  
//                  Move the devices further apart and restart the boards
//                  if necessary.
//              3.  Data packet loss is not handled in this program.
************************************************************************************/

#include "Gateway.h"


// Variable Declarations
extern char paTable[];
extern char paTableLen;

char rxBuffer[10];              // receiver packet buffer
char len=PSIZE-1;               // Length of the received packet

// Function Definitions
void UART_Initialize(void)
// Function to initialize UART communication to transfer data  
// to a PC via a RS-232 connection (38,400 baud rate).
{
	P2SEL |= BIT4+BIT5;                     // Set UC0TXD and UC0RXD to 
                                                // transmit and receive data
	UCA0CTL0 = 0;                           // USCI_A0 control register
	UCA0CTL1 |= UCSSEL_2 + BIT0;            // Clock source SMCLK
	UCA0BR0=27;                             // 1 MHz, 38400 baud rate
	UCA0BR1=0;                              // 1 MHz, 38400 baud rate
	UCA0MCTL=0x94;                          // Modulation
	UCA0CTL1 &= ~UCSWRST;                   // Software reset
}	// end UART_Initialize

void UART_putchar(char c)
// Function to send a single byte of data, c, over to a PC using UART 
// communication via a RS232 connection.
{
	while (!(IFG2 & UCA0TXIFG));            // Wait for other character to transmit
	UCA0TXBUF = c;                          // TXBuffer is assigned c
}	// end UART_putchar

void Software_Delay(void)
// Software delay to allow for proper operation of receiver on start up and 
// after POR.
{
		for(int i=0;i<2500;i++)         // POR software delay
                  ;
}	// end Software_Delay

void TI_CC_Initialize()
// Function to support TI CC1100 (TI_CC) port initialization to support wireless 
// communication via SPI provided by the TI CC1100 chip which attaches
// to the transmitting TI MSP430 experimenter board.
// GDO0 to RX packet info from CCxxxx
{
	TI_CC_LED_PxDIR = TI_CC_LED1;           // Port 1 LED set to Outputs
	TI_CC_LED_PxOUT = TI_CC_LED1;           // Initialize, turn on LED1
	TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;     // Set interrupt to occur on the
                                            // on the falling edge 
                                            // (i.e, end of data packet)
	TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;    // Initialize, clear
                                            // TI_CC_GDO0_PIN bit which is
                                            //set after data packet transfer        
	TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;      // Enable interrupt to occur at
                                            // the end of a data packet
}	// end TI_CC_Initialize (Receiver version)

void Receiver_Initialization()
// Function to support MSP430 receiver initialization to support wireless 
// communication  via SPI provided by the TI CC1100 chip which attaches
// to the receiving TI MSP430 experimenter board.
{
	Software_Delay();                       // Start up/POR software delay
	TI_CC_SPISetup();                       // Initialize SPI port
	TI_CC_PowerupResetCCxxxx();             // Reset CCxxxx
	writeRFSettings();                      // Write RF settings to
                                            // configure registers
	TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);
                                            // Write PATABLE
	TI_CC_Initialize();                     // Initialize CCxxx on port 1.
	TI_CC_SPIStrobe(TI_CCxxx0_SRX);         // Initialize CCxxxx in RX mode.
                                            // Set receiving packets to
						// signal on GDO0 and wake CPU
 
}	//end Receiver_Initialization
  
void transmittoUART()
{
    TI_CC_LED_PxOUT ^=(TI_CC_LED1); // Toggle LED1 to confirm data receipt
  
    // Using the UART_putchar() function, send byte by byte to UAH Serial App.
    // Transmit header required for UAH Serial App.
    UART_putchar(0x55);  
    
    //checksum will be sent at the end of every single packet
    short int checksum = 0;
    
    // Transmit the Fahrenheit value of the temperature to UAH Serial APP
    for(int i=0; i<4; i++)
    {
       checksum = checksum ^ (rxBuffer[PDATAINDEX-1+i]) ; 
       UART_putchar(rxBuffer[PDATAINDEX-1+i]);
    }
    // Transmit the Celsius value of the temperature to UAH Serial APP
    for(int i=4; i<8; i++)
    {
     checksum = checksum ^ rxBuffer[PDATAINDEX-1+i] ;   
     UART_putchar(rxBuffer[PDATAINDEX-1+i]);
    }          
    UART_putchar(checksum);
}

void main (void)
{
	WDTCTL = WDTPW + WDTHOLD;     // Stop WDT       
	Receiver_Initialization();    // Initialization of the receiving MSP430 experimenter board	
	UART_Initialize();            // Initialize UART
   
    while(1)
    {
        _BIS_SR(LPM3_bits + GIE); // Enter LPM3, enable interrupts
        transmittoUART();         // reads the data from the rxbuffer and transmit to UART
    }
}	// end main

#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR (void)
// MSP430 experimenter board Port 1 interrupt vector service routine.
// GDO0 connected to port 1 pin transitions from 1 to 0 when the packet is received.
// 1->0 transition will initiate this ISR in which received packet is read out
// and stored to rxBuffer 
{
	if(P1IFG & TI_CC_GDO0_PIN)              // Command received from RF RX active
        {   
        if( RFReceivePacket(rxBuffer,&len))   // check if packet is received       
	    __bic_SR_register_on_exit(LPM3_bits + GIE);   // Exit LPM0  
	 // reset of TI_CC_GDO0_PIN bit which is set after data packet transfer  
        P1IFG &= ~TI_CC_GDO0_PIN;       
        }	// end if
        
}	// end MSP430 Port 1 ISR