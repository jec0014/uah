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

#include "Gateway.h"	// Header File

// Variable Declarations
extern char paTable[];
extern char paTableLen;

char txBuffer[10];  // transfer packet buffer
int temp_x;			// Temporary variable to store the ADC12 Sampled Value
long int IntDegF, IntDegC;	// Variables to store the Temperature values


// Function Definitions
void Timer_Initialize(void)
// Function to support Timer B initializaiton for ADC12 sampling of a temperature 
// Sensor with a frequency of 20 samples/s (i.e.,50ms intervals) 
//using the reset/set mode, ACLK in up mode.
{
    TBCCTL1 |= OUTMOD_7;        // Reset/Set output mode  
    TBCTL |= TBSSEL_1 + MC_1;   // ACLK in UP mode
    TBCCR0 = 1658;              // Set CCR0 = CCR1 + 20
    TBCCR1 = 1638;              // Set CCR0 to 50 ms
                                // 32768 Hz X 50 ms = 1638.4
}	// end Timer_Initialization - Timer B

void ADC_Initialization(void)
// Function to support Timer B initializaiton for ADC12 sampling of a temperature 
// Sensor with a frequency of 20 samples/s (i.e.,50ms intervals) 
// ADC_Setup is as follows:
// Enable the following in the ADC12CTL registers:
// ADC12CTL0:  ADC12 core, ADC12ON
//             ADC12 start conversion bit, ADC12SC
//             Conversion bit, ENC 
//             REFON and SHT0_8
//             Sample-hold-time in pulse mode w/ 64 ADC12CLK cycles, SHT0_4
//             & SHP in ADC12CTL1
//             Multiple samples & conversion, MSC  
//             SAMPCON signal is sourced from the sampling timer, SHP 
//             Sample-and-hold source : Timer B OUT1, SHS_3
{ 
	// Enable ADC12 core// 64 ADC12CLK cycles and 
    ADC12CTL0 |= ADC12ON +REFON+ SHT0_8 ;    
    // Enable pulse mode sample & hold time TimerB OUT1 for sample timing 
	ADC12CTL1 |= SHP + SHS_3;   
    // Select the channel 10 for on chip temperature sensor
	ADC12MCTL0 = 0x01A;			
	ADC12IE = 0x001;    // Enable interrupts: ADC12IFG.0 for ADC12MEM0
    // Enable ADC conversion - both conversion start&enable bits   
	ADC12CTL0 |= ADC12SC + ENC; 
}	// end ADC_Initialization

// RFTransmitPacket transmits the packet
void RFTransmitPacket()
{
    TI_CC_SPIWriteBurstReg(TI_CCxxx0_TXFIFO, txBuffer, PSIZE); // Write TX data      
    // The CC1100 won't transmit the contents of the FIFO until the state is
    // changed to TX state.  During configuration we placed it in RX state and
    // configured it to return to RX whenever it is done transmitting, so it is
    // in RX now.  Use the appropriate library function to change the state to TX.
    TI_CC_SPIStrobe(TI_CCxxx0_STX);    // Change state to TX, initiating
                                        // data transfer  

    while (!(TI_CC_GDO0_PxIN&TI_CC_GDO0_PIN));// Wait GDO0 to go hi -> sync TX'ed
    while (TI_CC_GDO0_PxIN&TI_CC_GDO0_PIN);   // Wait GDO0 to clear -> end of pkt
}
void sendData()
// Function to send Fahrenheit and Celsius values of temperature  
// from the transmitting TI MSP430 experimenter board.  
// To perform the data transfer, the data payload is moved to txBuffer and  
// the function RFTransmitPacket is called.
// The temperature measurements are stored as long int values which occupy
// four bytes each for a total of 8 bytes.
// The RFTransmitPacket function expects the following parameters: 
// 1) packet length (byte 0)
// 2) packet address (byte 1)
// 3) packet payload (byte 2 - byte 9)
// txBuffer[PLENGTHINDEX]       =       PSIZE-1 (Packet Array Length Index)
// txBuffer[PADDRINDEX]         =       0x1 (Packet Array Address Index)
// txBuffer[PDATAINDEX]         =       first byte of payload
//                                      (Packet Array Data Start Index) 
// constants used to access txBuffer array locations as defined in gateway.h:    
// PLENGTHINDEX                 =       0  (Packet Array Location 0)
// ADDRINDEX                    =       1  (Packet Array Location 1)
// PDATAINDEX                   =       2  (Packet Array Location 2, 
//                                          Data Start Index)
// PSIZE                        =       10 (Packet Array Size)
{    
    // Convert the sampled value from ADC12 in temp_x to Fahrenheit
    IntDegF = ((long)temp_x - 2519) * 761;	 
    IntDegF = IntDegF / 4096;			
    // Convert the sampled value from ADC12 in temp_x to Celsius    
    IntDegC = ((long)temp_x - 2692) * 423;	 
    IntDegC = IntDegC / 4096;
            
    char *Fahrenheit_Value = (char *)&IntDegF;	// character pointer to the IntDegF
    char *Celsius_Value = (char *)&IntDegC;		// character pointer to the IntDegC
        
		
	// Fill up the Local Buffer, txBuffer
	txBuffer[PLENGTHINDEX] = PSIZE-1;       // Packet length
	txBuffer[PADDRINDEX] = 0x1;             // Packet address
	  
    // As IntDegF and IntDegC are declared as a long integer, 
    // they are 4 bytes long. 
	for(int i=0; i<4; i++)
        // Fahrenheit value of the temperature
        txBuffer[PDATAINDEX+i] = Fahrenheit_Value[i];	
	for(int i=4; i<8; i++)
        // Celsius value of the temperature
        txBuffer[PDATAINDEX+i] = Celsius_Value[i-4];	
	
	// change the status of the CC1101 chip to IDLE mode
    TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);   
    RFTransmitPacket();                 // transmit the packet
        
    TI_CC_SPIStrobe(TI_CCxxx0_SRX);     // go back to Receives mode
    // reset of TI_CC_GDO0_PIN bit which is set after data packet transfer
    P1IFG &= ~TI_CC_GDO0_PIN;           
    TI_CC_LED_PxOUT ^=(TI_CC_LED1);	    // Flash LED1 after data packet
                                        // transfer  
}   // end function sendData

// Toggle the LED while actual transmission is happening
void  TI_CC_Initialize()
// Function to initialize ports for interfacing TI CC1100 (TI_CC) 
{
	TI_CC_LED_PxDIR = TI_CC_LED1;           // Port 1 LED set to Outputs
	TI_CC_LED_PxOUT = TI_CC_LED1;           // Initialize, turn on LED1
	TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;    // Initialize, clear 
                                            // TI_CC_GDO0_PIN bit which is
                                            //set after data packet transfer   
}	// end TI_CC_Initialize (Transmitter version)

void Transmitter_Initialization()
// Function to support MSP430 transmitter initialization to support wireless 
// communication  via SPI provided by the TI CC1100 chip which attaches
// to the transmitting TI MSP430 experimenter board.
{  
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
	Transmitter_Initialization();           
	ADC_Initialization();                   // ADC Initialization for 
                                            // temperature sampling        
	Timer_Initialize();                     // Timer B Setup for 
                                            // temperature sampling  
	while(1)
	{  
		_BIS_SR(LPM3_bits + GIE);       // Enter LPM0, enable interrupts        
		//Call the sendData function to send the packet to receiver(end) device 
                sendData();
                                               
	}  // end while 
}   // end main 

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR(void)   // ADC12 Interrupt Service Routine
// ADC12 interrupt vector service routine to read the ADC12 memory buffers 
//(i.e, ADC12MEM0, and assignment to the temporary values (i.e., temp_x)
{
	temp_x = ADC12MEM0;       // Move results from ADC12MEM0 to temp variable, temp_x     
	ADC12CTL0 &= ~ENC;        // Stop ADC conversion
	ADC12CTL0 |= ENC;         // Enable ADC Conversion

	__bic_SR_register_on_exit(LPM3_bits);   // Exit LPM0
}       // end MSP430 ADC12 ISR