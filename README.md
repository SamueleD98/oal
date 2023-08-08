# OAL - Obstacle Avoidance Library
The library allows computing waypoints to reach a goal, avoiding moving obstacles.  


## Main files
- DataStructs, groups the definition of the used structures:
    - VehicleInfo, keeps info such as the position and speed of the vehicle. The struct is meant to be upgradable according to the developments of the library.
    - ObstacleInfo, list of obstacles. Again, upgradable as needed.
    - Node, represents a possible waypoint with an estimated cost (time), the obstacle and vertex trace and a pointer to its parent node
    - Obstacle, has its id, heading, speed, dimension, bounding box info (max and safety bb ration wrt original dimension) and the vertexes position (locally). Also, a function computes the position for a specific time instant, according to speed and heading, and one computes the four vertexes' position with respect to the centre of the obstacle. For the moment I'm keeping the attributes as public to easily access them for the plots (as soon as I reach a definitive version I'll make them private)
    - Vertex, a vertex (abs frame) position and id wrt to the obstacle. Can also save the point of intercept (where and when) with the vehicle
- Header Path Planner
    - path_planner class definition. Public functions are just the constructor (needs the vehicle and obstacles info) and the ComputePath function that makes a stack of waypoints to reach a given goal (if possible).
- Code Path Planner, the implementations of the main functions:
    - ComputeInterceptPoints
    - CheckCollision
    - ComputePath
    - UpdateCosts, simply computes the costs according to the vehicle speed. It could easily be a Node function but, since depends on the vehicle speed and on the goal, I'd rather keep it here. Any thoughts?
    - FindVisibility, given the vehicle and some vxs position, finds which of them are visible from the vehicle itself.
- Test

## Plots
By saving some data during the execution I managed to write a Python script (should be in src) to visualize the position of the objects in the main temporal instants. Here is an example (the obstacles speed is 0):
![Screenshot from 2023-08-04 19-37-11](https://github.com/SamueleD98/oal/assets/28822110/49a516e2-9ec3-45a8-9d1b-cb6d716a0168)

The red dot is the vehicle. Also, the safety and maximum bounding box are shown.  

Here is an example with moving obstacles:
![Screenshot from 2023-08-05 13-12-59](https://github.com/SamueleD98/oal/assets/28822110/e671a79e-a710-4bd0-a516-be01c5f3d21d)  

In order to check the correctness of the CheckCollision function, I wrote a script to visualize the bounding boxes and the vehicle path. Here an example:
![Figure_1](https://github.com/SamueleD98/oal/assets/28822110/34b667d5-8ca8-4d2a-bca5-49d13a8e3098)



## Notes  

As expected, an obstacle going through the goal will crash into the vehicle (look below). Still I will further explore this situation to make sure it was not the vehicle to crash into the obstacle.
![Screenshot from 2023-08-05 12-58-38](https://github.com/SamueleD98/oal/assets/28822110/35e7df1a-1307-4d52-b4d0-e2e41c4beefc)  

About variables name: should I use the underscore (e.g. id_) even for structs with public attributes? (I need to fix most of the names since some of them are with the underscore and some are not)  

I'm checking the validity of some of the lib input (such as the obstacle dimension to be positive) by throwing 'std::invalid_argument' exceptions. Is it ok?  

For now, I'm keeping obstacle attributes as public since I need them to be easily accessible for the plots. In the final version, I will set them as private.  



## Current limits

- The vehicle is a dimensionless point
- The vehicle has to be faster than every vehicle
- Vehicle and obstacles speed/heading are considered constants
- ..

## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake --build ./ --target oal_test -j 6
