#ifndef MAP_H
#define MAP_H

// Standard include
#include <vector>
#include <fstream>
#include <algorithm>

// External include
#include "external/json.hpp"

using namespace std;

//Information about map and operations with it
class Map {
public:

    /**
    * Constructor
    */
    Map();

    /**
    * Destructor
    */
    virtual ~Map();

    // Load map data from file
    void load_data(string file_path);
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_x;
	vector<double> map_y;
	vector<double> map_s;
	vector<double> map_dx;
	vector<double> map_dy;
    int map_size;

    vector<double> getXY(double s, double d);
    vector<double> getFrenet(double x, double y, double theta);
    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y);
    int NextWaypoint(double x, double y, double theta);

    vector<int> lanes = {0, 1, 2};

    int first_lane = lanes.front();
    int last_lane = lanes.back();

};

#endif
