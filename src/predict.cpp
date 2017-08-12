#include <math.h>

#include "predict.h"

/**
 * Initializes Predict
 */

Predict::Predict(Map input_map) {
    map = input_map;
}

Predict::~Predict() {}

double Predict::velocity(double& ref_vel, int current_lane) {

    if (states.prev_size > 0) {
        states.car_s = states.end_path_s;
    }

    bool too_close = false;

    // find ref_v to use

    for (int i = 0; i < states.sensor_fusion.size(); i++) {
        // car is in my lane
        float d = states.sensor_fusion[i][6];

        if (d < (2 + 4 * current_lane + 2) && d > (2 + 4 * current_lane - 2)) {
            double vx = states.sensor_fusion[i][3];
            double vy = states.sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = states.sensor_fusion[i][5];

            // if using previous points can project s value out
            check_car_s += ((double)states.prev_size * .02 * check_speed);

            // check s values greater than mine and s gap
            if ((check_car_s > states.car_s) && ((check_car_s - states.car_s) < 30)) {
                ref_vel = 29.5;
            }
        }
    }

    return ref_vel;
}
