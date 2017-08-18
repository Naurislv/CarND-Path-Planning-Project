# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program 11. project. Path planning.

Project resulting [video](https://www.youtube.com/watch?v=mS8yJiU0lA0). Car can drive itself in a highway and make manouvers based on sensory fusion data.

![Result](images/result.gif)


__NOTE:__ Repository consists of this project implementation as well as GaussianNaiveBayes programmed from scratche in C++ and it is not related to actual path planner.

## Download Simulator.

You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Run

1. Clone this repo: `git clone https://github.com/Naurislv/P11-Path-Planning-Project.git`
2. Navigate to repo directory: `cd P11-Path-Planning-Project`
2. Run: `bash run.sh`

# Implementation

## Code structure:

Code is structured so that each part is responsible for specific functionality and could be reused throught project as well as called and manipulated from main.cpp. In this way it is much easier to understand functionality and add additional functionalities. Also preformance must be as best as possible so each variables should ba called and defined only once if they does not change in any time moment. That is why e.g. we need map.h, map.cpp which store all information about map and if we need any functionality related to map we call function without providing map coordinates.

Detailed structure of source code:

```
  src/ :: All source code for project
      Eigen-3.3 :: Source code for Eigen to be used right away from #include
    
      external/ :: external headers
          json.hpp :: used for various forms of data reading in json format
          spline.hpp :: similar to polynomial fit but guarantees that line will go through all points
    
      helpers.h :: header file for helpers.cpp
      helpers.cpp :: multiple helper function which may be needed in any of code component

      map.h :: header file for map.cpp
      map.cpp :: load map coordinate system from file and contains all necesarry map manipulation functions

      states.h :: contains all momentary data about vehicle sent from Simulator. Used as type for other code parts.

      vehicle.h :: header file for vehicle.cpp
      vehicle.cpp :: responsilbe for car to drive as controlled (speed, brake, lane) and smooth trajectories

      predict.h :: header file for predict.cpp
      predict.cpp :: responsible for lane and velocity predictions, take advantage of sensor fusion data
```

main.cpp is responsible only for calling other functions and WS stream to and from Simulator. At the start we define three variables: vehicle, states and predict and pass them all to WS onMessage. For vehicle and predict classes in constructor we also pass map to each of classes to update ot with necesarry knowledge about environment. Inside WS on message instead of defining new variables we assign all incoming values from Simulator to states variables and then pass those states to vehicles and predict classes. Then we only need to call predict.movement() which assign vehicle.goal_vel and vehicle.lane new values based on sensor fusion data. And last step we need to do is execute vehicle.apply_path() which assign new values to next_x_vals and next_y_vals. That's, this way of structuring code gives as very easy way to work with it.

Few constants you have to consider before starting program in main.cpp :

* vehicle.lane: initial car target lane (should be 1 for this simulator);
* vehicle.ref_vel: vehicle reference velocty (should be around 50 mil/h), for this implementation it's 48.9
* predict.max_front_distance: max fron distance car will start to brake when it'll see car in it's lane
* predict.min_front_distance: min fron distance car will start to brake when it'll see car in it's lane, as predict module will change distance dynamically in range for more effective driving
* predict.safe_back_distance: when it's safe to make turn - we also must check if car isn't behine us

## How it works

## Predict::movement()

First functional component main.cpp is calling is Predict::movement(). It updates goal_vel and current_lane with values based on sensor fusion data. Inside function basically there are two parts :

1. Cost applying to each possible behaviour:

  For each of the behavior there are actually two behaviours. One is responsible for adding little cost so that car might act for more efficient path planning and other is responsible to add very high cost to one behaviour if there is collision possibility so that later we does not make such movement.

  * keep_straight - if other car is in our lane we add cost of other cars future position (based on it's speed and prevous path size multiplied by update step 0.02) minus our position. This really gives us sense if our paths are crossing and if we should be worried. Nice thing about this cost function is that it not only predicts near collisions but also can be used to make decisions about steering long before we are close to this car.
  * free_left - similar to keep_straight except we do not check we car is too close that we should break because, well car is not in our lane.
  * free_right - similar to free_left

2. Based on cost we applied for each of behavour we now either apply them by changing current lane or current velocity.

Perhaps for future implementations I should split Predict::movement() function int two separate functions base on previous functionalities.

There are of-course few more tricks (like checking how much car is away from position it should be so it first would drive to there before making other action and so on) I didn't mention so that prediction would be robust. Check [source](src/predict.cpp) for more details.

## Vehicle::apply_path()

Is function is called right after we predict where we can and cannot go. It can of-course work without any prediction step but then it wont be safe to drive. This function basically create dots with small interval (0.02ms for this simulator) between pat we need to go, e.g. 2 map waypoints and destionation point in other lane. Other important role of this function is to create smooth transitions so path it create is not uncomfortable for driver.

It does that by using car turning angle and creates smooth transition. One important point is that we transform coordinates to cars perspective so that x axis is looking at the same direction as car, this makes math easier. We create vector of 5 points where first points is smooth transition from cars current future state to future state using previous mentioned tricks and then we append 3 more points extraxted from map using Frenet coordinates towards and convert them to XY.

We are using spline instead of polynomial fit which is much easier and comes with single header file. It creates from prevoious appended 5 points 50 points with interval of .02.

See [source](src/vehicle.cpp) for more info.

## Future

There are plenty of room for improvement. For example currently we are looking only for left and right actions based on where we are now but there are three lanes and we should also look at all lanes - maybe we should move 2 lanes to left which is free ? Also it would be nice if car would have states like prepare for left or prepare for right which would be usefull for braking or acelarating to get better position and make turn.

# Starter Guide

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
