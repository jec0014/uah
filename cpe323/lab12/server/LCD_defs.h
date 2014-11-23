/**************************************************************************/
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*                          LCD Symbols definitions                       */
/*                                                                        */
/*                             Softbaugh SBLCDA4                          */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/* MSP430 Teaching ROM                                                    */
/* Produced by: MSP430@UBI Group - www.msp430.ubi.pt                      */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*  See the details of the LCD descriptio in Annex B                      */
/*  SBLCDA4_Specification.pdf                                             */
/*                                                                        */
/*                                                                        */
/*                                                                        */
/*                                        Copyright by Texas Instruments  */
/**************************************************************************/


                              

#define P1_NULL LCDM3 = 0x00                       //    digit(1) (OFF)
#define P1_A0 LCDM3 = 0x5F                         //    digit(1) = "0" (ON)
#define P1_A1 LCDM3 = 0x06                         //    digit(1) = "1" (ON)
#define P1_A2 LCDM3 = 0x6B                         //    digit(1) = "2" (ON)
#define P1_A3 LCDM3 = 0x2F                         //    digit(1) = "3" (ON)
#define P1_A4 LCDM3 = 0x36                         //    digit(1) = "4" (ON)
#define P1_A5 LCDM3 = 0x3D                         //    digit(1) = "5" (ON)
#define P1_A6 LCDM3 = 0x7D                         //    digit(1) = "6" (ON)
#define P1_A7 LCDM3 = 0x07                         //    digit(1) = "7" (ON)
#define P1_A8 LCDM3 = 0x7F                         //    digit(1) = "8" (ON)
#define P1_A9 LCDM3 = 0x3F                         //    digit(1) = "9" (ON)
#define P1_DOT_ON LCDM3 |= 0x80                    //    digit(1) = "." (ON)
#define P1_DOT_OFF LCDM3 &= ~0x80                  //    digit(1) = "." (OFF)
#define P1_B1 LCDM3 = 0x59                        // Letter 'C'
#define P1_B2 LCDM3 = 0x71                        // Letter 'F'

#define P2_NULL LCDM4 = 0x00                       //    digit(2) (OFF)
#define P2_A0 LCDM4 = 0x5F                         //    digit(2) = "0" (ON)
#define P2_A1 LCDM4 = 0x06                         //    digit(2) = "1" (ON)
#define P2_A2 LCDM4 = 0x6B                         //    digit(2) = "2" (ON)
#define P2_A3 LCDM4 = 0x2F                         //    digit(2) = "3" (ON)
#define P2_A4 LCDM4 = 0x36                         //    digit(2) = "4" (ON)
#define P2_A5 LCDM4 = 0x3D                         //    digit(2) = "5" (ON)
#define P2_A6 LCDM4 = 0x7D                         //    digit(2) = "6" (ON)
#define P2_A7 LCDM4 = 0x07                         //    digit(2) = "7" (ON)
#define P2_A8 LCDM4 = 0x7F                         //    digit(2) = "8" (ON)
#define P2_A9 LCDM4 = 0x3F                         //    digit(2) = "9" (ON)
#define P2_DOT_ON LCDM4 |= 0x80                    //    digit(2) = "." (ON)
#define P2_DOT_OFF LCDM4 &= ~0x80                  //    digit(2) = "." (OFF)
#define P2_B1 LCDM4 = 0x33                         // degree sign


#define P3_NULL LCDM4 = 0x00                       //    digit(3) (OFF)
#define P3_A0 LCDM5 = 0x5F                         //    digit(3) = "0" (ON)
#define P3_A1 LCDM5 = 0x06                         //    digit(3) = "1" (ON)
#define P3_A2 LCDM5 = 0x6B                         //    digit(3) = "2" (ON)
#define P3_A3 LCDM5 = 0x2F                         //    digit(3) = "3" (ON)
#define P3_A4 LCDM5 = 0x36                         //    digit(3) = "4" (ON)
#define P3_A5 LCDM5 = 0x3D                         //    digit(3) = "5" (ON)
#define P3_A6 LCDM5 = 0x7D                         //    digit(3) = "6" (ON)
#define P3_A7 LCDM5 = 0x07                         //    digit(3) = "7" (ON)
#define P3_A8 LCDM5 = 0x7F                         //    digit(3) = "8" (ON)
#define P3_A9 LCDM5 = 0x3F                         //    digit(3) = "9" (ON)
#define P3_DOT_ON LCDM5 |= 0x80                    //    digit(3) = "." (ON)
#define P3_DOT_OFF LCDM5 &= ~0x80                  //    digit(3) = "." (OFF)
#define P3_B1 LCDM5 = 0x09                        //     equal sign = '='  

#define P4_NULL LCDM6 = 0x00                       //    digit(4) (OFF)
#define P4_A0 LCDM6 = 0x5F                         //    digit(4) = "0" (ON)
#define P4_A1 LCDM6 = 0x06                         //    digit(4) = "1" (ON)
#define P4_A2 LCDM6 = 0x6B                         //    digit(4) = "2" (ON)
#define P4_A3 LCDM6 = 0x2F                         //    digit(4) = "3" (ON)
#define P4_A4 LCDM6 = 0x36                         //    digit(4) = "4" (ON)
#define P4_A5 LCDM6 = 0x3D                         //    digit(4) = "5" (ON)
#define P4_A6 LCDM6 = 0x7D                         //    digit(4) = "6" (ON)
#define P4_A7 LCDM6 = 0x07                         //    digit(4) = "7" (ON)
#define P4_A8 LCDM6 = 0x7F                         //    digit(4) = "8" (ON)
#define P4_A9 LCDM6 = 0x3F                         //    digit(4) = "9" (ON)
#define P4_DOT_ON LCDM6 |= 0x80                    //    digit(4) = "." (ON)
#define P4_DOT_OFF LCDM6 &= ~0x80                  //    digit(4) = "." (OFF)

#define P5_NULL LCDM7 = 0x00                       //    digit(5) (OFF)
#define P5_A0 LCDM7 = 0x5F                         //    digit(5) = "0" (ON)
#define P5_A1 LCDM7 = 0x06                         //    digit(5) = "1" (ON)
#define P5_A2 LCDM7 = 0x6B                         //    digit(5) = "2" (ON)
#define P5_A3 LCDM7 = 0x2F                         //    digit(5) = "3" (ON)
#define P5_A4 LCDM7 = 0x36                         //    digit(5) = "4" (ON)
#define P5_A5 LCDM7 = 0x3D                         //    digit(5) = "5" (ON)
#define P5_A6 LCDM7 = 0x7D                         //    digit(5) = "6" (ON)
#define P5_A7 LCDM7 = 0x07                         //    digit(5) = "7" (ON)
#define P5_A8 LCDM7 = 0x7F                         //    digit(5) = "8" (ON)
#define P5_A9 LCDM7 = 0x3F                         //    digit(5) = "9" (ON)
#define P5_DOT_ON LCDM7 |= 0x80                    //    digit(5) = "." (ON)
#define P5_DOT_OFF LCDM7 &= ~0x80                  //    digit(5) = "." (OFF)

#define P6_NULL LCDM8 = 0x00                       //    digit(6) (OFF)
#define P6_A0 LCDM8 = 0x5F                         //    digit(6) = "0" (ON)
#define P6_A1 LCDM8 = 0x06                         //    digit(6) = "1" (ON)
#define P6_A2 LCDM8 = 0x6B                         //    digit(6) = "2" (ON)
#define P6_A3 LCDM8 = 0x2F                         //    digit(6) = "3" (ON)
#define P6_A4 LCDM8 = 0x36                         //    digit(6) = "4" (ON)
#define P6_A5 LCDM8 = 0x3D                         //    digit(6) = "5" (ON)
#define P6_A6 LCDM8 = 0x7D                         //    digit(6) = "6" (ON)
#define P6_A7 LCDM8 = 0x07                         //    digit(6) = "7" (ON)
#define P6_A8 LCDM8 = 0x7F                         //    digit(6) = "8" (ON)
#define P6_A9 LCDM8 = 0x3F                         //    digit(6) = "9" (ON)
#define P6_DOT_ON LCDM8 |= 0x80                    //    digit(6) = "." (ON)
#define P6_DOT_OFF LCDM8 &= ~0x80                  //    digit(6) = "." (OFF)

#define P7_NULL LCDM9 = 0x00                       //    digit(7) (OFF)
#define P7_A0 LCDM9 = 0x5F                         //    digit(7) = "0" (ON)
#define P7_A1 LCDM9 = 0x06                         //    digit(7) = "1" (ON)
#define P7_A2 LCDM9 = 0x6B                         //    digit(7) = "2" (ON)
#define P7_A3 LCDM9 = 0x2F                         //    digit(7) = "3" (ON)
#define P7_A4 LCDM9 = 0x36                         //    digit(7) = "4" (ON)
#define P7_A5 LCDM9 = 0x3D                         //    digit(7) = "5" (ON)
#define P7_A6 LCDM9 = 0x7D                         //    digit(7) = "6" (ON)
#define P7_A7 LCDM9 = 0x07                         //    digit(7) = "7" (ON)
#define P7_A8 LCDM9 = 0x7F                         //    digit(7) = "8" (ON)
#define P7_A9 LCDM9 = 0x3F                         //    digit(7) = "9" (ON)
#define P7_DOT_ON LCDM9 |= 0x80                    //    digit(7) = "." (ON)
#define P7_DOT_OFF LCDM9 &= ~0x80                  //    digit(7) = "." (OFF)
