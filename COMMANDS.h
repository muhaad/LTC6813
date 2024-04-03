

#ifndef COMMANDS_H  
#define COMMANDS_H

#include <stdint.h>

extern int wire_cut;        //indicates position of wire break . "0" indicates normal operation with no break in the isoSPI line

#define WRCFGA   0b0000000000000001      //Write Configuration Register Group A
#define WRCFGB   0b0000000000100100      //Write Configuration Register Group B
#define RDCFGA   0b0000000000000010      //Read Configuration Register Group A
#define RDCFGB   0b0000000000100110      //Read Configuration Register Group B
#define RDCVA    0b0000000000000100      //Read Cell Voltage Register Group A
#define RDCVB    0b0000000000000110      //Read Cell Voltage Register Group B
#define RDCVC    0b0000000000001000      //Read Cell Voltage Register Group C
#define RDCVD    0b0000000000001010      //Read Cell Voltage Register Group D
#define RDCVE    0b0000000000001001      //Read Cell Voltage Register Group E
#define RDCVF    0b0000000000001011      //Read Cell Voltage Register Group F
#define RDAUXA   0b0000000000001100      //Read Auxiliary Register Group A
#define RDAUXB   0b0000000000001110      //Read Auxiliary Register Group B
#define RDAUXC   0b0000000000001101      //Read Auxiliary Register Group C
#define RDAUXD   0b0000000000001111      //Read Auxiliary Register Group D
#define RDSTATA  0b0000000000010000      //Read Status Register Group A
#define RDSTATB  0b0000000000010010      //Read Status Register Group B 
#define WRSCTRL  0b0000000000010100      //Write S Control Register Group 
#define WRPWM    0b0000000000100000      //Write PWM Register Group 
#define WRPSB    0b0000000000011100      //Write PWM/S Control Register Group B 
#define RDSCTRL  0b0000000000010110      //Read S Control Register Group 
#define RDPWM    0b0000000000100010      //Read PWM Register Group 
#define RDPSB    0b0000000000011110      //Read PWM/S Control Register Group B 
#define STSCTRL  0b0000000000011001      //Start S Control Pulsing and Poll Status
#define CLRSCTRL 0b0000000000011000      //Clear S Control Register Group 
#define CLRCELL  0b0000011100010001      //Clear Cell Voltage Register Groups 
#define CLRAUX   0b0000011100010010      //Clear Auxiliary Register Groups
#define CLRSTAT  0b0000011100010011      //Clear Status Register Groups 
#define PLADC    0b0000011100010100      //Poll ADC Conversion Status 
#define DIAGN    0b0000011100010101      //Diagnose MUX and Poll Status 
#define WRCOMM   0b0000011100100001      //Write COMM Register Group 
#define RDCOMM   0b0000011100100010      //Read COMM Register Group 
#define STCOMM   0b0000011100100011      //Start I 2C/SPI Communication 
#define MUTE     0b0000000000101000      //Mute Discharge
#define UNMUTE   0b0000000000101001      //Unmute Discharge 
                
#define ADCV     0b0000001001100000      //Start Start Cell Voltage ADC Conversion and Poll Status


#endif