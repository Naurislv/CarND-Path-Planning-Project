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
	
	cout << "Map size: " << map_size << endl;
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

double Map::distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Map::ClosestWaypoint(double x, double y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < map_x.size(); i++)
	{
		double maps_x = map_x[i];
		double maps_y = map_y[i];
		double dist = distance(x, y, maps_x, maps_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int Map::NextWaypoint(double x, double y, double theta)
{

	int closestWaypoint = ClosestWaypoint(x,y);

	double maps_x = map_x[closestWaypoint];
	double maps_y = map_y[closestWaypoint];

	double heading = atan2( (maps_y-y),(maps_x-x) );

	double angle = abs(theta-heading);

	if(angle > M_PI/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta)
{
	int next_wp = NextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = map_x.size()-1;
	}

	double n_x = map_x[next_wp]-map_x[prev_wp];
	double n_y = map_y[next_wp]-map_y[prev_wp];
	double x_x = x - map_x[prev_wp];
	double x_y = y - map_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-map_x[prev_wp];
	double center_y = 2000-map_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(map_x[i],map_y[i],map_x[i+1],map_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}