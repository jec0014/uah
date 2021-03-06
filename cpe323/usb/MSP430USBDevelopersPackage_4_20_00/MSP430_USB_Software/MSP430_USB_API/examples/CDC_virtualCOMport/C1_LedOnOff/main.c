/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*  
 * ======== main.c ========
 * LED Control Demo:
 *
 * This example implements a simple command-line interface, where the command 
 * is ended by pressing �return�.  It accepts four "commands":
 * "LED ON"
 * "LED OFF"
 * "LED TOGGLE - SLOW"
 * "LED TOGGLE � FAST"
 +----------------------------------------------------------------------------+
 * Please refer to the Examples Guide for more details.
 * ---------------------------------------------------------------------------*/
#include <string.h>

#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                 // USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"

// Function declarations
uint8_t retInString (char* string);
void initTimer(void);

// Global flags set by events
volatile uint8_t bCDCDataReceived_event = FALSE; // Indicates data has been rx'ed
                                              // without an open rx operation

#define MAX_STR_LENGTH 64
char wholeString[MAX_STR_LENGTH] = ""; // Entire input str from last 'return'

// Set/declare toggle delays
uint16_t SlowToggle_Period = 20000 - 1;
uint16_t FastToggle_Period = 1000 - 1;

/*  
 * ======== main ========
 */
void main (void)
{
    WDT_A_hold(WDT_A_BASE); // Stop watchdog timer

    // Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
#ifndef DRIVERLIB_LEGACY_MODE
    PMM_setVCore(PMM_CORE_LEVEL_2);
#else
    PMM_setVCore(PMM_BASE, PMM_CORE_LEVEL_2);
#endif

    initPorts();           // Config GPIOS for low-power (output low)
    initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO=32kHz
    initTimer();           // Prepare timer for LED toggling
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

    __enable_interrupt();  // Enable interrupts globally
    
    while (1)
    {
        uint8_t i;
        
        // Check the USB state and directly main loop accordingly
        switch (USB_connectionState())
        {
            // This case is executed while your device is enumerated on the
            // USB host
            case ST_ENUM_ACTIVE:
            
                // Enter LPM0 (can't do LPM3 when active)
                __bis_SR_register(LPM0_bits + GIE);
                _NOP(); 
                // Exit LPM on USB receive and perform a receive operation
                
                // If true, some data is in the buffer; begin receiving a cmd
                if (bCDCDataReceived_event){
 
                    // Holds the new addition to the string
                    char pieceOfString[MAX_STR_LENGTH] = "";
                    
                    // Holds the outgoing string
                    char outString[MAX_STR_LENGTH] = "";

                    // Add bytes in USB buffer to the string
                    cdcReceiveDataInBuffer((uint8_t*)pieceOfString,
                        MAX_STR_LENGTH,
                        CDC0_INTFNUM); // Get the next piece of the string

                    // Append new piece to the whole
                    strcat(wholeString,pieceOfString);

                    // Echo back the characters received
                    cdcSendDataInBackground((uint8_t*)pieceOfString,
                        strlen(pieceOfString),CDC0_INTFNUM,0);

                    // Has the user pressed return yet?
                    if (retInString(wholeString)){
                    
                        // Compare to string #1, and respond
                        if (!(strcmp(wholeString, "LED ON"))){
                        
                            // Turn off timer; no longer toggling LED
                            TIMER_A_stop(TIMER_A0_BASE);
                            
                            // Turn on LED P1.0
                            GPIO_setOutputHighOnPin(LED_PORT, LED_PIN);
                            
                            // Prepare the outgoing string
                            strcpy(outString,"\r\nLED is ON\r\n\r\n");
                            
                            // Send the response over USB
                            cdcSendDataInBackground((uint8_t*)outString,
                                strlen(outString),CDC0_INTFNUM,0);
                                
                        // Compare to string #2, and respond
                        } else if (!(strcmp(wholeString, "LED OFF"))){
                        
                            // Turn off timer; no longer toggling LED
                            TIMER_A_stop(TIMER_A0_BASE);
                            
                            // Turn off LED P1.0
                            GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);
                            
                            // Prepare the outgoing string
                            strcpy(outString,"\r\nLED is OFF\r\n\r\n");
                            
                            // Send the response over USB
                            cdcSendDataInBackground((uint8_t*)outString,
                                strlen(outString),CDC0_INTFNUM,0);
                                
                        // Compare to string #3, and respond
                        } else if (!(strcmp(wholeString, "LED TOGGLE - SLOW"))){
                        
                            // Turn off timer while changing toggle period
                            TIMER_A_stop(TIMER_A0_BASE);
                            
                            // Set timer period for slow LED toggle
                            TIMER_A_configureUpMode(TIMER_A0_BASE,
                                TIMER_A_CLOCKSOURCE_ACLK,
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,
                                SlowToggle_Period,
                                TIMER_A_TAIE_INTERRUPT_DISABLE,
                                TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
                                TIMER_A_DO_CLEAR
                                );
                                
                            // Start timer for toggling
                            TIMER_A_startCounter(TIMER_A0_BASE,
                                TIMER_A_UP_MODE);
                                
                            // Prepare the outgoing string
                            strcpy(outString,
                                "\r\nLED is toggling slowly\r\n\r\n");
                                
                            // Send the response over USB
                            cdcSendDataInBackground((uint8_t*)outString,
                                strlen(outString),CDC0_INTFNUM,0);
                                
                        // Compare to string #4, and respond
                        } else if (!(strcmp(wholeString,"LED TOGGLE - FAST"))){
                        
                            // Turn off timer
                            TIMER_A_stop(TIMER_A0_BASE);
                            
                            // Turn off timer while changing toggle period
                            TIMER_A_configureUpMode(TIMER_A0_BASE,
                                TIMER_A_CLOCKSOURCE_ACLK,
                                TIMER_A_CLOCKSOURCE_DIVIDER_1,
                                FastToggle_Period,
                                TIMER_A_TAIE_INTERRUPT_DISABLE,
                                TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
                                TIMER_A_DO_CLEAR
                                );
                                
                            // Start timer for toggling
                            TIMER_A_startCounter(TIMER_A0_BASE,
                                TIMER_A_UP_MODE);
                                
                            // Prepare the outgoing string
                            strcpy(outString,
                                "\r\nLED is toggling fast\r\n\r\n");
                                
                            // Send the response over USB
                            cdcSendDataInBackground((uint8_t*)outString,
                                strlen(outString),CDC0_INTFNUM,0);
                                
                        // Handle other
                        } else {
                        
                            // Prepare the outgoing string
                            strcpy(outString,"\r\nNo such command!\r\n\r\n");
                            
                            // Send the response over USB
                            cdcSendDataInBackground((uint8_t*)outString,
                                strlen(outString),CDC0_INTFNUM,0);
                        }
                        
                        // Clear the string in preparation for the next one
                        for (i = 0; i < MAX_STR_LENGTH; i++){
                            wholeString[i] = 0x00;
                        }
                    }
                    bCDCDataReceived_event = FALSE;
                }
                break;
                
            // These cases are executed while your device is disconnected from
            // the host (meaning, not enumerated); enumerated but suspended
            // by the host, or connected to a powered hub without a USB host
            // present.
            case ST_PHYS_DISCONNECTED:
            case ST_ENUM_SUSPENDED:
            case ST_PHYS_CONNECTED_NOENUM_SUSP:
            
                //Turn off LED P1.0
                GPIO_setOutputLowOnPin(LED_PORT, LED_PIN);
                __bis_SR_register(LPM3_bits + GIE);
                _NOP();
                break;

            // The default is executed for the momentary state
            // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
            // seconds.  Be sure not to enter LPM3 in this state; USB
            // communication is taking place here, and therefore the mode must
            // be LPM0 or active-CPU.
            case ST_ENUM_IN_PROGRESS:
            default:;
        }

    }  // while(1)
} // main()

/*  
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
#ifndef DRIVERLIB_LEGACY_MODE
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
#else
            UCS_clearFaultFlag(UCS_BASE, UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_BASE, UCS_DCOFFG);
            SFR_clearInterrupt(SFR_BASE, SFR_OSCILLATOR_FAULT_INTERRUPT);

#endif
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; //clear bus error flag
            USB_disable(); //Disable
    }
}



/*  
 * ======== retInString ========
 */
// This function returns true if there's an 0x0D character in the string; and if
// so, it trims the 0x0D and anything that had followed it.
uint8_t retInString (char* string)
{
    uint8_t retPos = 0,i,len;
    char tempStr[MAX_STR_LENGTH] = "";

    strncpy(tempStr,string,strlen(string));  // Make a copy of the string
    len = strlen(tempStr);
    
    // Find 0x0D; if not found, retPos ends up at len
    while ((tempStr[retPos] != 0x0A) && (tempStr[retPos] != 0x0D) &&
           (retPos++ < len)) ;

    // If 0x0D was actually found...
    if ((retPos < len) && (tempStr[retPos] == 0x0D)){
        for (i = 0; i < MAX_STR_LENGTH; i++){ // Empty the buffer
            string[i] = 0x00;
        }
        
        //...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);
        
        //...and tell the calling function that we did so
        return ( TRUE) ;
        
    // If 0x0D was actually found...
    } else if ((retPos < len) && (tempStr[retPos] == 0x0A)){
        // Empty the buffer
        for (i = 0; i < MAX_STR_LENGTH; i++){
            string[i] = 0x00;
        }
        
        //...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);
        
        //...and tell the calling function that we did so
        return ( TRUE) ;
    } else if (tempStr[retPos] == 0x0D){
        for (i = 0; i < MAX_STR_LENGTH; i++){  // Empty the buffer
            string[i] = 0x00;
        }
        // ...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);
        // ...and tell the calling function that we did so
        return ( TRUE) ;
    } else if (retPos < len){
        for (i = 0; i < MAX_STR_LENGTH; i++){  // Empty the buffer
            string[i] = 0x00;
        }
        
        //...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);
        
        //...and tell the calling function that we did so
        return ( TRUE) ;
    }

    return ( FALSE) ; // Otherwise, it wasn't found
}

/*
 * ======== initTimer ========
 */
void initTimer (void)
{
    // Start timer
    TIMER_A_clearTimerInterruptFlag(TIMER_A0_BASE);

    TIMER_A_configureUpMode(TIMER_A0_BASE,
        TIMER_A_CLOCKSOURCE_ACLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        0,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
        );
}

/*
 * ======== TIMER1_A0_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not found!
#endif
{
    GPIO_toggleOutputOnPin(LED_PORT, LED_PIN);
}

//Released_Version_4_20_00
