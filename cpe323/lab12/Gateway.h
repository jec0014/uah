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



// Modified by - Sunny Patel, Sjohn Chambers and Pamela Mazurkivich
// March 2013
// Wireless Gateway Receiver and Transmitter for MSP430 using CC1101
// Program for Dr. Aleksandar Milenkovic
// written by Sjohn Chambers, Sunny Patel, & Pam Mazurkivich January 2013
// This file contains the constants, variables, and functions to be included 
// with wireless gateway project.
//----------------------------------------------------------------------------

//constants used by wireless_gateway_transmit.c & wireless_gateway_receive.c
#define PSIZE 3                   //Packet Array Size     
#define PLENGTHINDEX 0             //Packet Array Length Index
#define PADDRINDEX 1               //Packet Array Address Index
#define PDATAINDEX 2               //Packet Array Data Start Index

//TI_CC_msp430 function definitions
#include "msp430xG46x.h"                     // Adjust this according to the
                                            // MSP430 device being used.

// SPI port definitions                     // Adjust the values for the chosen
#define TI_CC_SPI_USART0_PxSEL  P3SEL       // interfaces, according to the pin
#define TI_CC_SPI_USART0_PxDIR  P3DIR       // assignments indicated in the
#define TI_CC_SPI_USART0_PxIN   P3IN        // chosen MSP430 device datasheet.
#define TI_CC_SPI_USART0_SIMO   0x02
#define TI_CC_SPI_USART0_SOMI   0x04
#define TI_CC_SPI_USART0_UCLK   0x08

#define TI_CC_SPI_USART1_PxSEL  P4SEL
#define TI_CC_SPI_USART1_PxDIR  P4DIR
#define TI_CC_SPI_USART1_PxIN   P4IN
#define TI_CC_SPI_USART1_SIMO   0x08
#define TI_CC_SPI_USART1_SOMI   0x10
#define TI_CC_SPI_USART1_UCLK   0x20

#define TI_CC_SPI_USCIA0_PxSEL  P3SEL
#define TI_CC_SPI_USCIA0_PxDIR  P3DIR
#define TI_CC_SPI_USCIA0_PxIN   P3IN
#define TI_CC_SPI_USCIA0_SIMO   0x10
#define TI_CC_SPI_USCIA0_SOMI   0x20
#define TI_CC_SPI_USCIA0_UCLK   0x01

#define TI_CC_SPI_USCIA1_PxSEL  P7SEL
#define TI_CC_SPI_USCIA1_PxDIR  P7DIR
#define TI_CC_SPI_USCIA1_PxIN   P7IN
#define TI_CC_SPI_USCIA1_SIMO   0x02
#define TI_CC_SPI_USCIA1_SOMI   0x04
#define TI_CC_SPI_USCIA1_UCLK   0x08

#define TI_CC_SPI_USCIB0_PxSEL  P3SEL
#define TI_CC_SPI_USCIB0_PxDIR  P3DIR
#define TI_CC_SPI_USCIB0_PxIN   P3IN
#define TI_CC_SPI_USCIB0_SIMO   0x02
#define TI_CC_SPI_USCIB0_SOMI   0x04
#define TI_CC_SPI_USCIB0_UCLK   0x08

#define TI_CC_SPI_USCIB1_PxSEL  P3SEL
#define TI_CC_SPI_USCIB1_PxDIR  P3DIR
#define TI_CC_SPI_USCIB1_PxIN   P3IN
#define TI_CC_SPI_USCIB1_SIMO   0x02
#define TI_CC_SPI_USCIB1_SOMI   0x04
#define TI_CC_SPI_USCIB1_UCLK   0x08

#define TI_CC_SPI_USI_PxDIR     P1DIR
#define TI_CC_SPI_USI_PxIN      P1IN
#define TI_CC_SPI_USI_SIMO      0x40
#define TI_CC_SPI_USI_SOMI      0x80
#define TI_CC_SPI_USI_UCLK      0x20

#define TI_CC_SPI_BITBANG_PxDIR P5DIR
#define TI_CC_SPI_BITBANG_PxOUT P5OUT
#define TI_CC_SPI_BITBANG_PxIN  P5IN
#define TI_CC_SPI_BITBANG_SIMO  0x02
#define TI_CC_SPI_BITBANG_SOMI  0x04
#define TI_CC_SPI_BITBANG_UCLK  0x08


//----------------------------------------------------------------------------
//  These constants are used to identify the chosen SPI and UART interfaces.
//----------------------------------------------------------------------------
#define TI_CC_SER_INTF_NULL    0
#define TI_CC_SER_INTF_USART0  1
#define TI_CC_SER_INTF_USART1  2
#define TI_CC_SER_INTF_USCIA0  3
#define TI_CC_SER_INTF_USCIA1  4
#define TI_CC_SER_INTF_USCIB0  5
#define TI_CC_SER_INTF_USCIB1  6
#define TI_CC_SER_INTF_USI     7
#define TI_CC_SER_INTF_BITBANG 8


//hal_uart function definitions
void halUARTSetup(void);
void blastString(char[], unsigned int);

//CC1100-CC2500 function definitions
void writeRFSettings(void);
void RFSendPacket(char *, char);
char RFReceivePacket(char *rxBuffer, char *length);

//TI_CC_spi function definitions
void TI_CC_SPISetup(void);
void TI_CC_PowerupResetCCxxxx(void);
void TI_CC_SPIWriteReg(char, char);
void TI_CC_SPIWriteBurstReg(char, char*, char);
char TI_CC_SPIReadReg(char);
void TI_CC_SPIReadBurstReg(char, char *, char);
char TI_CC_SPIReadStatus(char);
void TI_CC_SPIStrobe(char);
void TI_CC_Wait(unsigned int);

//TI_CC_hardware_board definitions - no TI_CC__hardware_board.c file entries
#define TI_CC_LED_PxOUT         P2OUT
#define TI_CC_LED_PxDIR         P2DIR
#define TI_CC_LED1              0x02
#define TI_CC_LED2              0x04

#define TI_CC_SW_PxIN           P1IN
#define TI_CC_SW_PxIE           P1IE
#define TI_CC_SW_PxIES          P1IES
#define TI_CC_SW_PxIFG          P1IFG
#define TI_CC_SW1               0x01
#define TI_CC_SW2               0x02


#define TI_CC_GDO0_PxOUT        P1OUT
#define TI_CC_GDO0_PxIN         P1IN
#define TI_CC_GDO0_PxDIR        P1DIR
#define TI_CC_GDO0_PxIE         P1IE
#define TI_CC_GDO0_PxIES        P1IES
#define TI_CC_GDO0_PxIFG        P1IFG
#define TI_CC_GDO0_PIN          0x04

#define TI_CC_CSn_PxOUT         P4OUT
#define TI_CC_CSn_PxDIR         P4DIR
#define TI_CC_CSn_PIN           0x04


//----------------------------------------------------------------------------
// Select which port will be used for interface to CCxxxx
//----------------------------------------------------------------------------
#define TI_CC_RF_SER_INTF       TI_CC_SER_INTF_USART1  // Interface to CCxxxx


//TI_CC_CC1100-CC2500 definitions - no TI_CC_CC1100-CC2500.c file entries
// Configuration Registers
#define TI_CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define TI_CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define TI_CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define TI_CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define TI_CCxxx0_SYNC1        0x04        // Sync word, high byte
#define TI_CCxxx0_SYNC0        0x05        // Sync word, low byte
#define TI_CCxxx0_PKTLEN       0x06        // Packet length
#define TI_CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define TI_CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define TI_CCxxx0_ADDR         0x09        // Device address
#define TI_CCxxx0_CHANNR       0x0A        // Channel number
#define TI_CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define TI_CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define TI_CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define TI_CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define TI_CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define TI_CCxxx0_MDMCFG4      0x10        // Modem configuration
#define TI_CCxxx0_MDMCFG3      0x11        // Modem configuration
#define TI_CCxxx0_MDMCFG2      0x12        // Modem configuration
#define TI_CCxxx0_MDMCFG1      0x13        // Modem configuration
#define TI_CCxxx0_MDMCFG0      0x14        // Modem configuration
#define TI_CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define TI_CCxxx0_MCSM2        0x16        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM1        0x17        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_MCSM0        0x18        // Main Radio Cntrl State Machine config
#define TI_CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation config
#define TI_CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define TI_CCxxx0_AGCCTRL2     0x1B        // AGC control
#define TI_CCxxx0_AGCCTRL1     0x1C        // AGC control
#define TI_CCxxx0_AGCCTRL0     0x1D        // AGC control
#define TI_CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define TI_CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define TI_CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define TI_CCxxx0_FREND1       0x21        // Front end RX configuration
#define TI_CCxxx0_FREND0       0x22        // Front end TX configuration
#define TI_CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define TI_CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define TI_CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define TI_CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define TI_CCxxx0_FSTEST       0x29        // Frequency synthesizer cal control
#define TI_CCxxx0_PTEST        0x2A        // Production test
#define TI_CCxxx0_AGCTEST      0x2B        // AGC test
#define TI_CCxxx0_TEST2        0x2C        // Various test settings
#define TI_CCxxx0_TEST1        0x2D        // Various test settings
#define TI_CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define TI_CCxxx0_SRES         0x30        // Reset chip.
#define TI_CCxxx0_SFSTXON      0x31        // Enable/calibrate freq synthesizer
#define TI_CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define TI_CCxxx0_SCAL         0x33        // Calibrate freq synthesizer & disable
#define TI_CCxxx0_SRX          0x34        // Enable RX.
#define TI_CCxxx0_STX          0x35        // Enable TX.
#define TI_CCxxx0_SIDLE        0x36        // Exit RX / TX
#define TI_CCxxx0_SAFC         0x37        // AFC adjustment of freq synthesizer
#define TI_CCxxx0_SWOR         0x38        // Start automatic RX polling sequence
#define TI_CCxxx0_SPWD         0x39        // Enter pwr down mode when CSn goes hi
#define TI_CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define TI_CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define TI_CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define TI_CCxxx0_SNOP         0x3D        // No operation.

// Status registers
#define TI_CCxxx0_PARTNUM      0x30        // Part number
#define TI_CCxxx0_VERSION      0x31        // Current version number
#define TI_CCxxx0_FREQEST      0x32        // Frequency offset estimate
#define TI_CCxxx0_LQI          0x33        // Demodulator estimate for link quality
#define TI_CCxxx0_RSSI         0x34        // Received signal strength indication
#define TI_CCxxx0_MARCSTATE    0x35        // Control state machine state
#define TI_CCxxx0_WORTIME1     0x36        // High byte of WOR timer
#define TI_CCxxx0_WORTIME0     0x37        // Low byte of WOR timer
#define TI_CCxxx0_PKTSTATUS    0x38        // Current GDOx status and packet status
#define TI_CCxxx0_VCO_VC_DAC   0x39        // Current setting from PLL cal module
#define TI_CCxxx0_TXBYTES      0x3A        // Underflow and # of bytes in TXFIFO
#define TI_CCxxx0_RXBYTES      0x3B        // Overflow and # of bytes in RXFIFO
#define TI_CCxxx0_NUM_RXBYTES  0x7F        // Mask "# of bytes" field in _RXBYTES

// Other memory locations
#define TI_CCxxx0_PATABLE      0x3E
#define TI_CCxxx0_TXFIFO       0x3F
#define TI_CCxxx0_RXFIFO       0x3F

// Masks for appended status bytes
#define TI_CCxxx0_LQI_RX       0x01        // Position of LQI byte
#define TI_CCxxx0_CRC_OK       0x80        // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
#define TI_CCxxx0_WRITE_BURST  0x40
#define TI_CCxxx0_READ_SINGLE  0x80
#define TI_CCxxx0_READ_BURST   0xC0

