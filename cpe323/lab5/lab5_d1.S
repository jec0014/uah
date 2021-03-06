;****************************************************************************
; TI Experimenter board demo, blinking leds LED1 and LED2 (msp430FG4618)
;
; Description: Toggle P2.1 and P2.2 by xoring P2.1 and P2.2 inside a loop. 
;              The leds are connected to P2.1 and P2.2 and are on when
;              P2.1=1 and P2.2=1; 
;              The LEDs are initialized P2.1 to be off, and P2.2 to be on;
;              Clocks: ACLK = 32.768kHz, MCLK = SMCLK = default DCO (~1 MHz)
;
;                 MSP430xG461x
;             -----------------
;         /|\|                 |
;          | |                 |
;          --|RST              |
;            |             P2.2|-->LED1(GREEN)
;            |             P2.1|-->LED2(YELLOW)
;
; Alex Milenkovich, milenkovic@computer.org
;****************************************************************************
#include  <msp430xG46x.h>
;----------------------------------------------------------------------------
            RSEG    CSTACK                  ; Define stack segment
;----------------------------------------------------------------------------
            RSEG    CODE                    ; Assemble to Flash memory
;----------------------------------------------------------------------------
RESET       mov.w   #SFE(CSTACK),SP         ; Initialize stack pointer
StopWDT     mov.w   #WDTPW+WDTHOLD,&WDTCTL  ; Stop WDT
SetupP2     bis.b   #006h,&P2DIR            ; Set P2.1&P2.2 to output
                                            ; direction (0000_0110)
            bis.b   #002h,&P2OUT            ; Set P2OUT to 0x0000_0010 
                                            ;(LED2 is on, LED1 is off) 		
MainLoop    xor.b   #006h,&P2OUT            ; Toggle P2.1&P2.2 using XOR
Wait        mov.w   #050000,R15             ; Delay to R15
L1          dec.w   R15                     ; Decrement R15
            jnz     L1                      ; Delay over?
            jmp     MainLoop                ; Again
                                            ;
;----------------------------------------------------------------------------
            COMMON  INTVEC                  ; Interrupt Vectors
;----------------------------------------------------------------------------
            ORG     RESET_VECTOR            ; MSP430 RESET Vector
            DW      RESET                   ;
            END
;****************************************************************************
