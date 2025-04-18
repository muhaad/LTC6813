

#ifndef HEADER_H  
#define HEADER_H

#include <stdint.h>

void measure_temp(bool open_wire_check = false);

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

#endif

