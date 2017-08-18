#include "vehicle.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(Map input_map) {
    map = input_map;
}
Vehicle::~Vehicle() {}

void Vehicle::apply_path(
    vector<double> &next_x_vals,
    vector<double> &next_y_vals
) {
    // This is where splite fitted points will go in
    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = states.car_x; // reference x
    double ref_y = states.car_y; // reference y
    double ref_yaw = hlp::deg2rad(states.car_yaw); // reference yaw

    // With this operation we deal with new paths starting points
    // for next iteration making path transition from new prev path
    // to new path smooth
    if (states.prev_size < 2) {
        // Use two points that make the path tanget to the car
        double prev_car_x = states.car_x - cos(states.car_yaw);
        double prev_car_y = states.car_y - sin(states.car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(states.car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(states.car_y);
    } else {
        // Use the prev path's end point as starting reference

        // Redefine reference state as previous path end point
        ref_x = states.previous_path_x[states.prev_size - 1];
        ref_y = states.previous_path_y[states.prev_size - 1];

        double ref_x_prev = states.previous_path_x[states.prev_size - 2];
        double ref_y_prev = states.previous_path_y[states.prev_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        // Use two points that make the path tangent to the prevous path end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // In Frenet add evenly 30m spaced points ahead of the starting reference
    vector<double> next_wp0 = map.getXY(states.car_s + 30, 2 + 4 * lane);
    vector<double> next_wp1 = map.getXY(states.car_s + 60, 2 + 4 * lane);
    vector<double> next_wp2 = map.getXY(states.car_s + 90, 2 + 4 * lane);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // To set car to 0 in it's local coordinate system, so basically
    // if it was shifted 30 degrees left it now see points from it's
    // perspective
    for(int i = 0; i < ptsx.size(); i++)
    {
        // shift car reference angle to 0 degrees
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // create a spline
    tk::spline spl;

    // set (x, y) points to the spline
    spl.set_points(ptsx, ptsy);

    // Start with all of the previous path points from last time
    for (int i = 0; i < states.prev_size; i++)
    {
        next_x_vals.push_back(states.previous_path_x[i]);
        next_y_vals.push_back(states.previous_path_y[i]);
    }

    // Calculate how to break up spline points so that we travel at our desired reference velocity
    double target_x = 30.0;
    double target_y = spl(target_x);

    // !! ruff estimation of target line - linearization
    // this is where local coordinates kicks in
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points,
    // here we will always output 50 ponts

    for (int i = 1; i <= 50 - states.prev_size; i++) {
        
        // number of points from origin to target location
        double N = (target_dist / (.02 * goal_vel / 2.24));
        double x_point = x_add_on + (target_x) / N;
        double y_point = spl(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotatin it earlier
        // from local coordinates to global coordinates
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}
