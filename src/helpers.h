#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>

namespace hlp {
    double deg2rad(double x);
    double rad2deg(double x);
    void cout_vector(std::vector<double> input_data);

    // Found idx of value which is greater than
    int vecotor_next_idx(std::vector<double> vect, double val);
}

#endif