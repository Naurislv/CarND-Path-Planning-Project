#ifndef STATES_H
#define STATES_H

#include <vector>

using namespace std;

class States {  // Save and precompute some states for periodic usage
public:

	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
	vector<double> previous_path_x;
	vector<double> previous_path_y;
	double end_path_s;
	double end_path_d;
	vector<vector<double>> sensor_fusion;
    int prev_size;

	void set_previous_path_x (vector<double> vect) {
		// we cant assigned vector from outside otherwise
		previous_path_x = vect;
	}
	
	void set_previous_path_y (vector<double> vect) {
		// we cant assigned vector from outside otherwise
		previous_path_y = vect;
	}

	void set_sensor_fusion (vector<vector<double>> vect) {
		// we cant assigned vector from outside otherwise
		sensor_fusion = vect;
	}

    int set_prev_size() {
        prev_size = previous_path_x.size();
    }

};

#endif