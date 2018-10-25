# Path planning 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Term 3, Project 1: Path planning
### Keywords: Path planning, highway driving

---
In this project, the self-driving car should navigate safely around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.
The car is able to avoid collisions by adapting its velocity and changing lanes. It drives smoothly without sudden velocity or acceleration changes which would be unpleasant for passengers or physically impossible.

### Additional resources
The simulator is available [here (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).
The simulator simulates a 6-lane highway with traffic going in both directions. Only the three right lanes are of interest for this project.
The simulator will provide the car's localization and sensor fusion data.

Further, the highway is mapped to a sparse map list of waypoints (data/highway_map.txt) around the highway defining the lanes.
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.
The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

The starter code for this project is available in the [udacity repository (https://github.com/udacity/CarND-Path-Planning-Project)]

---

## Communication between the car and the simulator

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


#### Next waypoints (car -> simulator)
['next_x'] and ['next_y']: calculated ideal car trajectory (split into 50 points) 

## Details

The car uses a perfect controller and will visit every (x,y) point it receives in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.


## Model

The trajectory of the car is modeled in frenet coordinates by spline interpolation using the single header file from http://kluge.in-chemnitz.de/opensource/spline/,
For this, three points are calculated based on the current position and velocity of the car. Up to 50 points are then interpolated along this spline function. Lane changes and/or acceleration are easily incorporated.
Here, the velocity is only an indirect variable since the car "drives" from point to point in discreet 20ms. The velocity is therefore defined by v=s/t, where s is the distance between two points.
The calculated trajectory is added to previous, not yet processed points from the trajectory before. There will be always 50 points stored.

## Interaction with other vehicles

The car should not get too close to other cars. If it approaches a car in the same lane down to 30m, the flag "too_close" is set.
Then the car will attempt a lane change. It will first check if it is safe to go to the left lane. If this lane is blocked by another car up to 30m behind or before the car, it will not change to the lane on the left.
It will then check if a change to the right lane is possible following the same criteria. If this is also not possible, it will decelerate in order to avoid a collision.

## Improvements

1. The car could use more advanced logic to decide if a lane change to the left or to the right is preferable. Also it should have some "return" mode, so that it drives on the right lane when the highway is free.

2. When the car decelerates to avoid collisions (and also when it accelerates) it could sense the second car's velocity and set this one as reference velocity rather than the arbitrary value of 30 mph which might even be too high in some situations.

3. Lane changes of the other vehicles should be considered when changing lane in order to avoid other more complex collision scenarios.