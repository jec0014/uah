/* *********************************************************************************
// Description: Program samples the MSP430's analog temperature sensor (20 sps)
//              calculates the temperature in degrees Fahrenheit and Celsius, and
//              sends the calculated temperature wirelessly to a receiving node
//              using CC1101 RF transceiver.
// Platform:    TI Experimenter's board with MSP430FG4618/F2013 with CC1101
//
// Files:       BaseImplementation_tr.c
//              Gateway.c  - CC1101 hardware abstraction layer
//              Gateway.h  - CC1101 header file
//             
// Setup:       2 TI Experimenter boards (transmitter and receiver nodes).
//              This program runs on the transmitter node.
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
#include <string.h>
#include <stdio.h>
#include "Gateway.h"	// Header File
#include "LCD_defs.h"

// Variable Declarations
extern char paTable[];
extern char paTableLen;

#define SERVER_ADDRESS 0x1
#define CLIENT_ADDRESS 0x2

#define SW1_INDICATOR 0x02
#define LED2_TOGGLE_COMMAND 0x02

#define SW1 (BIT0&P1IN)
#define SW2 (BIT1&P1IN)

int temp_x;			// Temporary variable to store the ADC12 Sampled Value
long IntDegF, IntDegC;	// Variables to store the Temperature values

char *messages[] = { "Good job. You're doing something right.\r\n",
                      "You are recieving things correctly.\r\n",
                      "Don't forget to debounce your switch input.\r\n",
                      "If you're getting this, things are going alright.\r\n",
                      "Remember that your project proposals are due soon.\r\n",
                      "Boo\r\n",
                      "Blah Blah Blah\r\n"};
         

char messageCount=0;
char sw1IndicatorPacket[3];
char led2TogglePacket[3];
char rxBuffer[256];
char txBuffer[256];
char len;
long numReceived=1234567;

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
void turn_off()
{
   LCDM2 = 0x00;
 LCDM3 = 0x00;
 LCDM4 = 0x00;
 LCDM5 = 0x00;
 LCDM6 = 0x00;
 LCDM7 = 0x00;
 LCDM8 = 0x00;
 LCDM9 = 0x00;
 LCDM10 = 0x00;
 LCDM11 = 0x00;
 LCDM12 = 0x00;
 LCDM13 = 0x00;
}
void LCD_Initialize(void)
{
  WDTCTL = WDTPW + WDTHOLD;					// Stop watchdog timer
  // setup clocks
  FLL_CTL0 |= XCAP18PF;                     // Set load capacitance for xtal
  SCFI0 |= FN_2;                            // x2 DCO, 4MHz nominal DCO
  SCFQCTL = 74;                             // (74+1) x 32768 = 2.45Mhz
  // Select COM1-COM2-COM3
  P5DIR |= 0x1E;                                        // Ports P5.2, P5.3 and P5.4 as outputs
  P5SEL |= 0x1E;                                        // Ports P5.2, P5.3 and P5.4 as special function (COM1, COM2 and COM3)
  
  // Active S4 to S25
  LCDAPCTL0 = LCDS24 | LCDS20 | LCDS16 | LCDS12 | LCDS8 | LCDS4;
    
  // LCDA configuration 
  LCDACTL = LCDFREQ_192 | LCD4MUX | LCDSON | LCDON;     // (ACLK = 32768)/192, 4-mux LCD, LCD_A on, Segments on
  LCDAVCTL0 = LCDCPEN;                                  // Charge pump enable
  LCDAVCTL1 = VLCD_3_44;                                // VLCD = 3,44 V
  turn_off(); 								//Switch OFF LED 0
} // end LCD_Config

void display()
{
  char outString[10];
  sprintf(outString, "%ld", numReceived);
  switch (outString[6])
  {
                  case '1': P1_A1;
                          break;
                  case '2': P1_A2;
                          break;
                  case '3': P1_A3;
                          break;
                  case '4': P1_A4;
                          break;
                  case '5': P1_A5;
                          break;
                  case '6': P1_A6;
                          break;
                  case '7': P1_A7;
                          break;
                  case '8': P1_A8;
                          break;
                  case '9': P1_A9;
                          break;
                  case '0': P1_A0;
                          break;
                  default: break;
  }
  switch (outString[5])
  {
                  case '1': P2_A1;
                          break;
                  case '2': P2_A2;
                          break;
                  case '3': P2_A3;
                          break;
                  case '4': P2_A4;
                          break;
                  case '5': P2_A5;
                          break;
                  case '6': P2_A6;
                          break;
                  case '7': P2_A7;
                          break;
                  case '8': P2_A8;
                          break;
                  case '9': P2_A9;
                          break;
                  case '0': P2_A0;
                          break;
                  default: break;
  }
  switch (outString[4])
  {
                  case '1': P3_A1;
                          break;
                  case '2': P3_A2;
                          break;
                  case '3': P3_A3;
                          break;
                  case '4': P3_A4;
                          break;
                  case '5': P3_A5;
                          break;
                  case '6': P3_A6;
                          break;
                  case '7': P3_A7;
                          break;
                  case '8': P3_A8;
                          break;
                  case '9': P3_A9;
                          break;
                  case '0': P3_A0;
                          break;
                  default: break;
  }
  switch (outString[3])
  {
                  case '1': P4_A1;
                          break;
                  case '2': P4_A2;
                          break;
                  case '3': P4_A3;
                          break;
                  case '4': P4_A4;
                          break;
                  case '5': P4_A5;
                          break;
                  case '6': P4_A6;
                          break;
                  case '7': P4_A7;
                          break;
                  case '8': P4_A8;
                          break;
                  case '9': P4_A9;
                          break;
                  case '0': P4_A0;
                          break;
                  default: break;
  }
  switch (outString[2])
  {
                  case '1': P5_A1;
                          break;
                  case '2': P5_A2;
                          break;
                  case '3': P5_A3;
                          break;
                  case '4': P5_A4;
                          break;
                  case '5': P5_A5;
                          break;
                  case '6': P5_A6;
                          break;
                  case '7': P5_A7;
                          break;
                  case '8': P5_A8;
                          break;
                  case '9': P5_A9;
                          break;
                  case '0': P5_A0;
                          break;
                  default: break;
  }
  switch (outString[1])
  {
                  case '1': P6_A1;
                          break;
                  case '2': P6_A2;
                          break;
                  case '3': P6_A3;
                          break;
                  case '4': P6_A4;
                          break;
                  case '5': P6_A5;
                          break;
                  case '6': P6_A6;
                          break;
                  case '7': P6_A7;
                          break;
                  case '8': P6_A8;
                          break;
                  case '9': P6_A9;
                          break;
                  case '0': P6_A0;
                          break;
                  default: break;
  }
  switch (outString[0])
  {
                  case '1': P7_A1;
                          break;
                  case '2': P7_A2;
                          break;
                  case '3': P7_A3;
                          break;
                  case '4': P7_A4;
                          break;
                  case '5': P7_A5;
                          break;
                  case '6': P7_A6;
                          break;
                  case '7': P7_A7;
                          break;
                  case '8': P7_A8;
                          break;
                  case '9': P7_A9;
                          break;
                  case '0': P7_A0;
                          break;
                  default: break;
  }
}

void main (void)
{
	WDTCTL = WDTPW + WDTHOLD;               // Stop WDT
        LCD_Initialize();
    // Initialization of the transmitting MSP430 experimenter board
          _EINT();                      // Enable interrupts
        P1IE |= 0x03;                 // Enable P1 interrupt for bit 0 and 1
        P1IES |= 0x03;                // Set interrupt call to falling edge
        P1IFG &= ~(0x03);             // Clear interrupt flags
          P2DIR |= 0x06; 		// Set P2.1 and P2.2 to output (0000_0110) 
         P2OUT = 0x06;			// Set P2OUT to 0000_0010b
	Transmitter_Initialization();           
        
        display();
        
        led2TogglePacket[PLENGTHINDEX] = PSIZE - 1;
        led2TogglePacket[PADDRINDEX] = CLIENT_ADDRESS;
        led2TogglePacket[PDATAINDEX] = LED2_TOGGLE_COMMAND;
        
        led2TogglePacket[PLENGTHINDEX] = PSIZE - 1;
        led2TogglePacket[PADDRINDEX] = CLIENT_ADDRESS;
        led2TogglePacket[PDATAINDEX] = SW1_INDICATOR;
        
	while(1)
	{  
		_BIS_SR(LPM3_bits + GIE);       // Enter LPM0, enable interrupts        
                                               
	}  // end while 
}   // end main 

#pragma vector = PORT1_VECTOR
__interrupt void switch_ISR (void)
{
  P1IFG &= ~(0x03);            // clear interrupt flags
  if (SW1 == 0)                 // if SW2 is pressed
  {
    int i;
    for(i = 0; i < 0x2000; i++) //debounce delay
    {
      asm("nop");
    }
    if (SW1 == 0)
    {
      txBuffer[PLENGTHINDEX] = strlen(messages[messageCount]) + 2;
      txBuffer[PADDRINDEX] = CLIENT_ADDRESS;
      strcpy(txBuffer+PDATAINDEX, messages[messageCount]);
      RFSendPacket(txBuffer, strlen(messages[messageCount]) + 3);
      messageCount = (messageCount + 1) % 7;
    }
               
  }
  if (SW2 == 0)                 // if SW2 is pressed
  {
    int i;
    for(i = 0; i < 0x2000; i++) //debounce delay
    {
      asm("nop");
    }
    if (SW2 == 0)
    {
      RFSendPacket(led2TogglePacket, PSIZE);
    }
               
  }
     if(P1IFG & TI_CC_GDO0_PIN)              // Command received from RF RX active
     {   
        if( RFReceivePacket(rxBuffer,&len))   // check if packet is received       
	{
          
            if(rxBuffer[0] == SERVER_ADDRESS) //Make sure this message is intended for the server
            {
            
              if(rxBuffer[1] == LED2_TOGGLE_COMMAND && len==2)
              {
                  P2OUT ^= 0x02;
              }
                        
              else if(len >=5)
              {
                
                  numReceived = ((long)rxBuffer[1] << 0)  + ((long)rxBuffer[2] << 8 ) + ((long)rxBuffer[3] << 16 ) + ((long)rxBuffer[4] << 24 );
                  display();
              }
              
            }
          
           // if(rxBuffer[1] == 0x01)
          //  {
            //    RFSendPacket(messages[messageCount], strlen(messages[messageCount]));
           //     messageCount = (messageCount + 1) % 7;
           // }

        }
	P1IFG &= ~(TI_CC_GDO0_PIN); // reset of TI_CC_GDO0_PIN bit which is set after data packet transfer  
             
    }
}
