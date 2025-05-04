

#ifndef HEADER_H  
#define HEADER_H

#include <stdint.h>

void measure_temp(bool open_wire_check = false);


void poll_ADC(uint16_t command, bool curr_measure = false);     //curr_measure selects whether a current measurement is taken while polling the ADC (for synchronous Current and voltage measurements to determine cell internal resistance)

template <size_t rows, size_t cols> inline void min_max(float arr_2D[rows][cols], float* min, float* max) {   //finds the max and min values in any static 2D array of floats
    for (size_t i = 0; i < rows; ++i) {                                                                       //min and max must be initalized to sensible values beforehand
        for (size_t j = 0; j < cols; ++j) {
            if (arr_2D[i][j] < *min) {
                *min = arr_2D[i][j];
            }
            if (arr_2D[i][j] > *max) {
                *max = arr_2D[i][j];
            }
        }
    }
}

template <size_t length> inline int search(float arr[length], float value) {       //searches a sorted list for the nearest element and returns it index. 

float dist = std::abs(value - arr[0]);
    int best_index = 0;
    for (size_t i = 1; i < length; ++i) {
        float new_dist = std::abs(value - arr[i]);
        if (new_dist < dist) {
            dist = new_dist;
            best_index = i;
        } else {
            break; 
        }
    }

    return best_index;
}

#endif

