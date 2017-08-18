#ifndef PREDICT_H
#define PREDICT_H

#include <vector>

// Local includes
#include "states.h"
#include "map.h"

using namespace std;

class Predict {
public:

    /**
    * Constructor
    */
    Predict(Map input_map);

    /**
    * Destructor
    */
    virtual ~Predict();

    Map map;
    States states;

    double movement(double& goal_vel, double ref_vel, int& current_lane);

    int max_front_distance = 30;
    int min_front_distance = 5;
    double allowed_distance = max_front_distance;
    int safe_back_distance = 10;

};

#endif