//----------------------------------------------------------------------------
//  Description:  This file contains functions that configure the CC1100/2500
//  device.
//
//  Demo Application for MSP430/CC1100-2500 Interface Code Library v1.0
//
//  K. Quiring
//  Texas Instruments, Inc.
//  July 2006
//  IAR Embedded Workbench v3.42A
// Wireless Gateway Receiver and Transmitter for MSP430 using CC1101
// Program for Dr. Aleksandar Milenkovic
// written by Sjohn Chambers, Sunny Patel, & Pam Mazurkivich January 2013
// This file contains the constants, variables, and functions to be included 
// with wireless gateway project.
//----------------------------------------------------------------------------

#include "Gateway.h"

//hal_uart function definitions
//----------------------------------------------------------------------------
// Functions for sending debug strings to a UART.  Although the same interfaces
// defined for SPI (USARTn, USCIxn, USI, and bit-bang) could be used for UART,
// only USART0/1 are defined.
//----------------------------------------------------------------------------

void halUARTSetup(void)
{
  P2SEL |= 0x010;                           // P2.4 = USCI_A0 TXD
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 0x6D;                           // 1MHz 9600
  UCA0BR1 = 0x00;                           // 1MHz 9600
  UCA0MCTL = 4;                             // Modulation
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
}

void blastString(char string[], unsigned int len)
{
  volatile unsigned int i;

  for(i=0;i<len;i++)
  {
    while (!(IFG2 & UCA0TXIFG));            // USART1 TX buffer ready?
    UCA0TXBUF  = string[i];                 // Output character
  }
}

//CC1100-CC2500 function definitions
#define TI_CC_RF_FREQ  915  // 315, 433, 868, 915, 2400


#if TI_CC_RF_FREQ == 915                          // 915 MHz
// Product = CC1100
// Crystal accuracy = 40 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 540.000000 kHz
// Deviation = 0.000000
// Return state:  Return to RX state upon leaving either TX or RX
// Datarate = 250.000000 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 915.000000 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (11) Serial Clock
void writeRFSettings(void)
{
    // Write register settings
    TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG2,   0x0B); // GDO2 output pin config.
    TI_CC_SPIWriteReg(TI_CCxxx0_IOCFG0,   0x06); // GDO0 output pin config.
    TI_CC_SPIWriteReg(TI_CCxxx0_PKTLEN,   0xFF); // Packet length.
    TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL1, 0x05); // Packet automation control.
    TI_CC_SPIWriteReg(TI_CCxxx0_PKTCTRL0, 0x05); // Packet automation control.
    TI_CC_SPIWriteReg(TI_CCxxx0_ADDR,     0x01); // Device address.
    TI_CC_SPIWriteReg(TI_CCxxx0_CHANNR,   0x00); // Channel number.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL1,  0x0B); // Freq synthesizer control.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREQ2,    0x23); // Freq control word, high byte
    TI_CC_SPIWriteReg(TI_CCxxx0_FREQ1,    0x31); // Freq control word, mid byte.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREQ0,    0x3B); // Freq control word, low byte.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG4,  0x2D); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG3,  0x3B); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG2,  0x73); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG1,  0x22); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_MDMCFG0,  0xF8); // Modem configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en)
    TI_CC_SPIWriteReg(TI_CCxxx0_MCSM1 ,   0x3F); //MainRadio Cntrl State Machine
    TI_CC_SPIWriteReg(TI_CCxxx0_MCSM0 ,   0x18); //MainRadio Cntrl State Machine
    TI_CC_SPIWriteReg(TI_CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
    TI_CC_SPIWriteReg(TI_CCxxx0_BSCFG,    0x1C); //  Bit synchronization config.
    TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL2, 0xC7); // AGC control.
    TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL1, 0x00); // AGC control.
    TI_CC_SPIWriteReg(TI_CCxxx0_AGCCTRL0, 0xB2); // AGC control.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREND1,   0xB6); // Front end RX configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_FREND0,   0x10); // Front end RX configuration.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(TI_CCxxx0_TEST2,    0x88); // Various test settings.
    TI_CC_SPIWriteReg(TI_CCxxx0_TEST1,    0x31); // Various test settings.
    TI_CC_SPIWriteReg(TI_CCxxx0_TEST0,    0x0B); // Various test settings.
}

// PATABLE (0 dBm output power)
extern char paTable[] = {0x50};
extern char paTableLen = 1;

#endif

//-----------------------------------------------------------------------------
//  void RFSendPacket(char *txBuffer, char size)
//
//  DESCRIPTION:
//  This function transmits a packet with length up to 63 bytes.  To use this
//  function, GD00 must be configured to be asserted when sync word is sent and
//  de-asserted at the end of the packet, which is accomplished by setting the
//  IOCFG0 register to 0x06, per the CCxxxx datasheet.  GDO0 goes high at
//  packet start and returns low when complete.  The function polls GDO0 to
//  ensure packet completion before returning.
//
//  ARGUMENTS:
//      char *txBuffer
//          Pointer to a buffer containing the data to be transmitted
//
//      char size
//          The size of the txBuffer
//-----------------------------------------------------------------------------
void RFSendPacket(char *txBuffer, char size)
{
  TI_CC_SPIStrobe(TI_CCxxx0_SIDLE);  
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_TXFIFO, txBuffer, size); // Write TX data

  // The CC1100 won't transmit the contents of the FIFO until the state is
  // changed to TX state.  During configuration we placed it in RX state and
  // configured it to return to RX whenever it is done transmitting, so it is
  // in RX now.  Use the appropriate library function to change the state to TX.
  TI_CC_SPIStrobe(TI_CCxxx0_STX);           // Change state to TX, initiating
                                            // data transfer  

  while (!(TI_CC_GDO0_PxIN&TI_CC_GDO0_PIN));// Wait GDO0 to go hi -> sync TX'ed
  while (TI_CC_GDO0_PxIN&TI_CC_GDO0_PIN);   // Wait GDO0 to clear -> end of pkt
  
  TI_CC_SPIStrobe(TI_CCxxx0_SRX);
}

//-----------------------------------------------------------------------------
//  char RFReceivePacket(char *rxBuffer, char *length)
//
//  DESCRIPTION:
//  Receives a packet of variable length (first byte in the packet must be the
//  length byte).  The packet length should not exceed the RXFIFO size.  To use
//  this function, APPEND_STATUS in the PKTCTRL1 register must be enabled.  It
//  is assumed that the function is called after it is known that a packet has
//  been received; for example, in response to GDO0 going low when it is
//  configured to output packet reception status.
//
//  The RXBYTES register is first read to ensure there are bytes in the FIFO.
//  This is done because the GDO signal will go high even if the FIFO is flushed
//  due to address filtering, CRC filtering, or packet length filtering.
//
//  ARGUMENTS:
//      char *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      char *length
//          Pointer to location where length will be returned
//
//  RETURN VALUE:
//      char
//          0x80:  CRC OK
//          0x00:  CRC NOT OK (or no pkt was put in the RXFIFO due to filtering)
//-----------------------------------------------------------------------------
char RFReceivePacket(char *rxBuffer, char *length)
{
  char status[2];
  char pktLen;
  
  if ((TI_CC_SPIReadStatus(TI_CCxxx0_RXBYTES) & TI_CCxxx0_NUM_RXBYTES))
  {
    // Use the appropriate library function to read the first byte in the
    // RX FIFO, which is the length of the packet (the total remaining bytes
    // in this packet after reading this byte).  
    // Hint:  how many bytes are being retrieved?  One or multiple?  
    pktLen = TI_CC_SPIReadReg(TI_CCxxx0_RXFIFO);
    
    // Use the appropriate library function to read the rest of the packet into
    // rxBuffer (i.e., read pktLen bytes out of the FIFO)
    // Hint:  how many bytes are being retrieved?  One or multiple?  
    TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, rxBuffer, pktLen); // Pull data
    *length = pktLen;                       // Return the actual size

    // Our initialization code configured this CC1100 to append two status
    // bytes to the end of the received packets.  These bytes are still in the
    // FIFO.  Use the appropriate library function to read these two bytes into
    // the status array
    TI_CC_SPIReadBurstReg(TI_CCxxx0_RXFIFO, status, 2);

    return (char)(status[TI_CCxxx0_LQI_RX]&TI_CCxxx0_CRC_OK);
  }
  else
    return 0;                               // Error
}

//TI_CC_spi function definitions
//----------------------------------------------------------------------------
//  void TI_CC_SPISetup(void)
//
//  DESCRIPTION:
//  Configures the assigned interface to function as a SPI port and
//  initializes it.
//----------------------------------------------------------------------------
//  void TI_CC_SPIWriteReg(char addr, char value)
//
//  DESCRIPTION:
//  Writes "value" to a single configuration register at address "addr".
//----------------------------------------------------------------------------
//  void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Writes values to multiple configuration registers, the first register being
//  at address "addr".  First data byte is at "buffer", and both addr and
//  buffer are incremented sequentially (within the CCxxxx and MSP430,
//  respectively) until "count" writes have been performed.
//----------------------------------------------------------------------------
//  char TI_CC_SPIReadReg(char addr)
//
//  DESCRIPTION:
//  Reads a single configuration register at address "addr" and returns the
//  value read.
//----------------------------------------------------------------------------
//  void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
//
//  DESCRIPTION:
//  Reads multiple configuration registers, the first register being at address
//  "addr".  Values read are deposited sequentially starting at address
//  "buffer", until "count" registers have been read.
//----------------------------------------------------------------------------
//  char TI_CC_SPIReadStatus(char addr)
//
//  DESCRIPTION:
//  Special read function for reading status registers.  Reads status register
//  at register "addr" and returns the value read.
//----------------------------------------------------------------------------
//  void TI_CC_SPIStrobe(char strobe)
//
//  DESCRIPTION:
//  Special write function for writing to command strobe registers.  Writes
//  to the strobe at address "addr".
//----------------------------------------------------------------------------


// Delay function. # of CPU cycles delayed is similar to "cycles". Specifically,
// it's ((cycles-15) % 6) + 15.  Not exact, but gives a sense of the real-time
// delay.  Also, if MCLK ~1MHz, "cycles" is similar to # of useconds delayed.
void TI_CC_Wait(unsigned int cycles)
{
  while(cycles>15)                          // 15 cycles consumed by overhead
    cycles = cycles - 6;                    // 6 cycles consumed each iteration
}


// SPI port functions

#if TI_CC_RF_SER_INTF == TI_CC_SER_INTF_USART1


void TI_CC_SPISetup(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_CSn_PxDIR |= TI_CC_CSn_PIN;         // /CS disable

  ME2 |= USPIE1;                            // Enable USART1 SPI mode
  UCTL1 |= CHAR + SYNC + MM;                // 8-bit SPI Master **SWRST**
  UTCTL1 |= CKPL + SSEL1 + SSEL0 + STC;     // SMCLK, 3-pin mode
  UBR01 = 0x02;                             // UCLK/2
  UBR11 = 0x00;                             // 0
  UMCTL1 = 0x00;                            // No modulation
  TI_CC_SPI_USART1_PxSEL |= TI_CC_SPI_USART1_SIMO | TI_CC_SPI_USART1_SOMI | TI_CC_SPI_USART1_UCLK;
                                            // SPI option select
  TI_CC_SPI_USART1_PxDIR |= TI_CC_SPI_USART1_SIMO + TI_CC_SPI_USART1_UCLK;
                                            // SPI TXD out direction
  UCTL1 &= ~SWRST;                          // Initialize USART state machine
}

void TI_CC_SPIWriteReg(char addr, char value)
{
    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
    IFG2 &= ~URXIFG1;                       // Clear flag
    U1TXBUF = addr;                         // Send address
    while(!(U1TCTL&TXEPT));                 // Wait for TX to finish
    U1TXBUF = value;                        // Load data for TX after addr
    while(!(U1TCTL&TXEPT));                 // Wait for end of addr TX
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

void TI_CC_SPIWriteBurstReg(char addr, char *buffer, char count)
{
    char i;

    TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;      // /CS enable
    while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
    U1TXBUF = addr | TI_CCxxx0_WRITE_BURST; // Send address
    while (!(IFG2&UTXIFG1));                // Wait for TX to finish
    for (i = 0; i < count; i++)
    {
      U1TXBUF = buffer[i];                  // Send data
      while (!(IFG2&UTXIFG1));              // Wait for TX to finish
    }
   
    while(!(U1TCTL&TXEPT));
    TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;       // /CS disable
}

char TI_CC_SPIReadReg(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
 
  U1TXBUF = (addr | TI_CCxxx0_READ_SINGLE); // Send address
  while(!(U1TCTL&TXEPT));                   // Wait for addr to be sent
  U1TXBUF = 0;                              // Load dummy byte for TX after addr
  while(!(U1TCTL&TXEPT));                   // Wait for end of dummy byte TX
  x = U1RXBUF;                              // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIReadBurstReg(char addr, char *buffer, char count)
{
  unsigned int i;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  
  U1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  while (!(IFG2&UTXIFG1));                  // Wait for TXBUF ready
  U1TXBUF = 0;                              // Dummy write to read 1st data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  //while (!(IFG2&URXIFG1));                  // Wait for end of addr byte TX
  while(!(U1TCTL&TXEPT));
  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    U1TXBUF = 0;                            //Initiate next data RX, meanwhile..
    buffer[i] = U1RXBUF;                    // Store data from last data RX
    while (!(IFG2&URXIFG1));                // Wait for end of data RX
  }
  buffer[count-1] = U1RXBUF;                // Store last RX byte in buffer
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

char TI_CC_SPIReadStatus(char addr)
{
  char x;

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  
  U1TXBUF = (addr | TI_CCxxx0_READ_BURST);  // Send address
  
  while(!(U1TCTL&TXEPT));                   // Wait for addr to be sent
  
  U1TXBUF = 0;                              // Dummy write so we can read data
  
  while(!(U1TCTL&TXEPT));                   // Wait for end of dummy byte TX
  x = U1RXBUF;                              // Read data
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable

  return x;
}

void TI_CC_SPIStrobe(char strobe)
{
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  U1TXBUF = strobe;                         // Send strobe
  // Strobe addr is now being TX'ed
  while(!(U1TCTL&TXEPT));                   // Wait for end of addr TX
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}

void TI_CC_PowerupResetCCxxxx(void)
{
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;
  TI_CC_Wait(30);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;
  TI_CC_Wait(45);

  TI_CC_CSn_PxOUT &= ~TI_CC_CSn_PIN;        // /CS enable
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);// Wait for CCxxxx ready
  U1TXBUF = TI_CCxxx0_SRES;                 // Send strobe
  // Strobe addr is now being TX'ed
  IFG2 &= ~URXIFG1;                         // Clear flag
  while (!(IFG2&URXIFG1));                  // Wait for end of addr TX
  while (TI_CC_SPI_USART1_PxIN&TI_CC_SPI_USART1_SOMI);
  TI_CC_CSn_PxOUT |= TI_CC_CSn_PIN;         // /CS disable
}



#endif