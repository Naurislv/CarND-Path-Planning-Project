#include "helpers.h"

namespace hlp{
    
    // Convert degrees to radians
    double deg2rad(double x) {
        return (x * M_PI) / 180;
    }

    // Convert radians to degrees
    double rad2deg(double x) {
        return x * 180 / M_PI;
    }

    // Print vector
    void cout_vector(std::vector<double> input_data){
        for (auto i = input_data.begin(); i != input_data.end(); ++i)
            std::cout << *i << ' ';
        
        std::cout << std::endl;
    }

    // Found idx of value which is greater than
    int vecotor_next_idx(std::vector<double> vect, double val) {
        // vect must be sorted
        int ret_val = 0;
        
        for (int i; i < vect.size(); i++) {
            if (vect[i] > val) {
                ret_val = i;
                break;
            }
        }

        return ret_val;
    }

}