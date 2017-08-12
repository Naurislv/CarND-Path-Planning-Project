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

    double velocity(double& ref_vel, int current_lane);

};

#endif