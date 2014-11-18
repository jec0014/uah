/* *********************************************************************************
// Description: This client program interfaces with another msp430 acting as a server.
                When SW2 is pressed on the client board, LED2 will toggle on the
                server board. When SW2 is pressed on the server board, LED2 will
                will toggle on the client board.
// Platform:    TI Experimenter's board with MSP430FG4618/F2013 with CC1101
//
// Files:       Lab12_D2.c
//              Gateway.c  - CC1101 hardware abstraction layer
//              Gateway.h  - CC1101 header file
//             
// Setup:       2 TI Experimenter boards (server and client nodes).
//              This program runs on the client node.
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
#include <string.h>

#include "Gateway.h"	// Header File

#define SW1 (BIT0&P1IN)
#define SW2 (BIT1&P1IN)

#define SERVER_ADDRESS 0x1
#define CLIENT_ADDRESS 0x2

#define LED2_TOGGLE_COMMAND 0x02

// Variable Declarations
extern char paTable[];
extern char paTableLen;

char led2TogglePacket[3];
char rxBuffer[256];
char len;

// Function Definitions

void Software_Delay(void)
// Software delay to allow for proper operation of receiver on start up and 
// after POR.
{
		for(int i=0;i<2500;i++)         // POR software delay
                  ;
}

// Toggle the LED while actual transmission is happening
void  TI_CC_Initialize()
// Function to initialize ports for interfacing TI CC1100 (TI_CC) 
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

void Transmitter_Initialization()
// Function to support MSP430 transmitter initialization to support wireless 
// communication  via SPI provided by the TI CC1100 chip which attaches
// to the transmitting TI MSP430 experimenter board.
{  
        Software_Delay();
	TI_CC_SPISetup();                       // Initialize SPI port
	TI_CC_PowerupResetCCxxxx();             // Reset CCxxxx
	writeRFSettings();                      // Write RF settings to config reg
	TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen); 
                                            // Write PATABLE
	TI_CC_Initialize();                     // Initialize CCxxx on port 1.
	TI_CC_SPIStrobe(TI_CCxxx0_SRX);         // Initialize CCxxxx in RX mode.
}	// end Transmitter_Initialization


void main (void)
{
	WDTCTL = WDTPW + WDTHOLD;               // Stop WDT
    // Initialization of the transmitting MSP430 experimenter board
          _EINT();                      // Enable interrupts
        P1IE |= 0x03;                 // Enable P1 interrupt for bit 0 and 1
        P1IES |= 0x03;                // Set interrupt call to falling edge
        P1IFG &= ~(0x03);             // Clear interrupt flags
          P2DIR |= 0x06; 		// Set P2.1 and P2.2 to output (0000_0110) 
         P2OUT = 0x06;			// Set P2OUT to 0000_0010b
	Transmitter_Initialization();      
        
        //Go ahead and set up the packet for toggleing LED2 on the server board
        //as it will not change.
        led2TogglePacket[PLENGTHINDEX] = PSIZE - 1;  
        led2TogglePacket[PADDRINDEX] = SERVER_ADDRESS;
        led2TogglePacket[PDATAINDEX] = LED2_TOGGLE_COMMAND;
        
	while(1)
	{  
		_BIS_SR(LPM3_bits + GIE);       // Enter LPM0, enable interrupts        
                                               
	}  // end while 
}   // end main 

#pragma vector = PORT1_VECTOR
__interrupt void switch_ISR (void)
{
  P1IFG &= ~(0x02);            // clear SW2 IFG
  if (SW2 == 0)                 // if SW2 is pressed
  {
    int i;
    for(i = 0; i < 0x2000; i++) //debounce delay
    {
      asm("nop");
    }
    if (SW2 == 0) //When SW2 is pressed, send command to toggle server LED2
    {
      RFSendPacket(led2TogglePacket, PSIZE);
    }
               
  }
     if(P1IFG & TI_CC_GDO0_PIN)              // Command received from RF RX active
     {   
        if( RFReceivePacket(rxBuffer,&len))   // check if packet is received       
	{
            
            if(rxBuffer[0] == CLIENT_ADDRESS) //Make sure this message is intended for the client
            {
          
              //if server sends LED2 toggle command, toggle LED2
              if(rxBuffer[1] == LED2_TOGGLE_COMMAND && len==2) //Messages longer than length 2 should be treated differently
              {
                  P2OUT ^= 0x02;
              }
              
            }
        }
	P1IFG &= ~(TI_CC_GDO0_PIN); // reset of TI_CC_GDO0_PIN bit which is set after data packet transfer  
             
    }
}
