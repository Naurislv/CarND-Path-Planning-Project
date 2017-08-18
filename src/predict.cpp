#include <math.h>

#include "predict.h"

/**
 * Initializes Predict
 */

Predict::Predict(Map input_map) {
    map = input_map;
}

Predict::~Predict() {}

double Predict::movement(double& goal_vel, double ref_vel, int& current_lane) {

    bool too_close = false;

    float keep_straight = 0;
    
    // Checking map if it's possible to make left turn
    float keep_left = 0;
    if (current_lane == map.first_lane) {
        keep_left = 1;
    }

    // Checking map if it's possible to make right turn
    float keep_right = 0;
    if (current_lane == map.last_lane) {
        keep_right = 1;
    }

    // Checking all possible maneuveres
    double front_car_speed = 0;
    
    for (int car_nr = 0; car_nr < states.sensor_fusion.size(); car_nr++) {
        float d = states.sensor_fusion[car_nr][6];

        double vx = states.sensor_fusion[car_nr][3];
        double vy = states.sensor_fusion[car_nr][4];
        
        // calculate vector length from origin which is magnitude of speed
        double check_speed = sqrt(vx * vx + vy * vy);
        double check_car_s = states.sensor_fusion[car_nr][5];

        // if using previous points can project s value outvards in time
        check_car_s += ((double)states.prev_size * .02 * check_speed);

        if (d < (2 + 4 * current_lane + 2) and d > (2 + 4 * current_lane - 2)) {
            // car is in my lane
            if (check_car_s > states.car_s) { // We are only interested in cars which is in front of us
                keep_straight += 1 / (check_car_s - states.car_s);  // Set cost to left turn if fusion see car there
            }

            // check s values greater than mine and s gap
            // check_car_s > states.car_s ---> If in fron of us
            // (check_car_s - states.car_s) < 25 ---> Less than 25 meters
            if ((check_car_s > states.car_s) && ((check_car_s - states.car_s) < allowed_distance)) {
                too_close = true;
                front_car_speed = check_speed;
            }
        } else if (d < (2 + 4 * (current_lane - 1) + 2) and d > (2 + 4 * (current_lane - 1) - 2)) {
            // car is in left lane

            if (check_car_s > states.car_s) { // We are only interested in cars which is in fron of us
                keep_left += 1 / (check_car_s - states.car_s);  // Set cost to left turn if fusion see car there
            }

            if ((check_car_s - states.car_s) < allowed_distance - 3 and
                (check_car_s - states.car_s) > - safe_back_distance) {
                    keep_left += 1;
            }
        } else if (d < (2 + 4 * (current_lane + 1) + 2) and d > (2 + 4 * (current_lane + 1) - 2)) {
            // car is in right lane

            if (check_car_s > states.car_s) { // We are only interested in cars which is in fron of us
                keep_right += 1 / (check_car_s - states.car_s);  // Set cost to rigth turn if fusion see car there
            }

            if ((check_car_s - states.car_s) < allowed_distance - 3 and
                (check_car_s - states.car_s) > - safe_back_distance) {
                keep_right += 1;
            }
        }
    }

    float delta_d = abs(states.car_d - (current_lane * 4 + 2));

    cout << "Left: " << keep_left << " " << "Right: " << keep_right << " "
         << "Straght: " << keep_straight << " " << "Front Distance: " << allowed_distance
         << endl;

    if (keep_right < keep_straight and delta_d < 1.5) {
        if (keep_right > keep_left){
            current_lane -= 1;
        } else {
            current_lane += 1;
        }
    } else if (keep_left < keep_straight and delta_d < 1.5) {
        if (keep_left > keep_right){
            current_lane += 1;
        } else {
            current_lane -= 1;
        }
    }

    if (too_close) { // Also make sure that car is not in the middle of lane transition

        goal_vel -= 0.8; // Stopping

        if (goal_vel < front_car_speed) {
            goal_vel = front_car_speed;
        }

        allowed_distance -= 0.3;
        if (allowed_distance < min_front_distance) {
            allowed_distance = min_front_distance;
        }

    } else if (goal_vel < ref_vel) {
        goal_vel += 0.6; // Accelerating
        allowed_distance += 0.1;
        
        if (allowed_distance > max_front_distance) {
            allowed_distance = max_front_distance;
        }
    } else if (allowed_distance < max_front_distance) {
        allowed_distance += 0.1;
        if (allowed_distance > max_front_distance) {
            allowed_distance = max_front_distance;
        }
    }
}
