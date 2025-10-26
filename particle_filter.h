#include <ap_fixed.h>
#include <hls_math.h>
#include <hls_vector.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdio.h>
#include "hls_stream.h"
#include "ap_int.h"

#define STATE_DIM 6
#define TMAX 100
#define N 1000

typedef float data_t;


void particle_filter
(
    bool start,
    data_t z_in[3],
    data_t sat_out[1],
    bool &read,
    bool &write,
    data_t x_out[STATE_DIM]
);


