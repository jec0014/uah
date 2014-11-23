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
 * ======== hal.h ========
 *
 * Device and board specific pins need to be configured here
 *
 */

// Only one of the following board defines should be enabled!
#define MSP430F5529LP       // F5529 LaunchPad
//#define MSPEXP430F5529     // F5529 Experimenters Board
//#define MSPTS430PN80USB    // FET target board
//#define MSPTS430RGC64USB   // FET target board
//#define MSPTS430PZ100USB   // FET target board


#ifdef MSPEXP430F5529
#define BUTTON1_PORT	GPIO_PORT_P1
#define BUTTON1_PIN		GPIO_PIN7
#define BUTTON1_VECTOR	PORT1_VECTOR

#define BUTTON2_PORT	GPIO_PORT_P2
#define BUTTON2_PIN		GPIO_PIN2
#define BUTTON2_VECTOR	PORT2_VECTOR
#endif

#if defined (MSPTS430RGC64USB) || defined (MSPTS430PN80USB)  || defined (MSPTS430PZ100USB)
#define BUTTON1_PORT	GPIO_PORT_P1
#define BUTTON1_PIN		GPIO_PIN6
#define BUTTON1_VECTOR	PORT1_VECTOR

#define BUTTON2_PORT	GPIO_PORT_P1
#define BUTTON2_PIN		GPIO_PIN7
#define BUTTON2_VECTOR	PORT1_VECTOR
#endif

#ifdef MSP430F5529LP
#define BUTTON1_PORT	GPIO_PORT_P1
#define BUTTON1_PIN		GPIO_PIN1
#define BUTTON1_VECTOR	PORT1_VECTOR

#define BUTTON2_PORT	GPIO_PORT_P2
#define BUTTON2_PIN		GPIO_PIN1
#define BUTTON2_VECTOR	PORT2_VECTOR
#endif


void initPorts(void);
void initClocks(uint32_t mclkFreq);
void initButtons(void);
//Released_Version_4_xx_xx_test
//Released_Version_4_20_00
