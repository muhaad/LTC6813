

#ifndef CONFIGURE_H  
#define CONFIGURE_H

#include <stdint.h>

#define num_boards 1;

int ADC_mode = 0;     //integer 0-7 to set ADC sampling frequency


            //       0      1      2     3     4     5     6      7
cell_conv_delays =  [1121,  1296,  2343, 3041, 4437, 7230, 12816, 201325];     //conversion time (in microseconds) of ADCs to measure all cells upon ADCV command based on ADC frequency
           //        27kHz  14kHz  7kHz   3kHz 2kHz  1kHz  422Hz  26Hz
cell_conv_delay = cell_conv_delays[ADC_mode];

            //       0      1      2     3     4     5     6      7
aux_conv_delays =  [1825, 2116, 3862, 5025, 7353, 12007, 21316, 335498];     //conversion time (in microseconds) of ADCs to measure all GPIO upon ADAX(D) commands based on ADC frequency
           //        27kHz  14kHz  7kHz   3kHz 2kHz  1kHz  422Hz  26Hz
aux_conv_delay = aux_conv_delays[ADC_mode];

stat_conv_delays =  [742, 858, 1556, 2022, 2953, 4814, 8538, 134211];     //conversion time (in microseconds) of ADCs to measure all GPIO upon ADAX(D) commands based on ADC frequency
           //        27kHz  14kHz  7kHz   3kHz 2kHz  1kHz  422Hz  26Hz
stat_conv_delay = stat_conv_delays[ADC_mode];


#endif