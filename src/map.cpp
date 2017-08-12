// Local include
#include "map.h"

using namespace std;

/**
 * Initializes Map
 */
Map::Map() {}

Map::~Map() {}

void Map::load_data(string file_path) {
	cout << "Loading map data from file.." << endl;
    // Load up map values for waypoint's x,y,s and d normalized normal vectors

	// Waypoint map to read from
	string map_file_ = file_path;
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_x.push_back(x);
		map_y.push_back(y);
		map_s.push_back(s);
		map_dx.push_back(d_x);
		map_dy.push_back(d_y);
	}

    map_size = map_x.size();
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d)
{
	int prev_wp = -1;

	while(s > map_s[prev_wp+1] && (prev_wp < (int)(map_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp + 1) % map_x.size();

	double heading = atan2((map_y[wp2]-map_y[prev_wp]),(map_x[wp2]-map_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map_s[prev_wp]);

	double seg_x = map_x[prev_wp]+seg_s*cos(heading);
	double seg_y = map_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}