# Monte Carlo Localization for pose estimations of a car 

## Project Overview
The robot has been kidnapped and transported to a new location. Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project, a 2-dimensional particle filter in C++ was implemented. The particle filter was given a map and some initial localization information (analogous to what a GPS would provide). At each time step, the filter will also get observation and control data. 

## Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|___data
|   |   
|   |   Debug Process.pdf
|   |   Running Simulator on windows and Code on Ubuntu.pdf
|   |   demo.gif
|
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```
## Running the Code
1. Clone the repository 
1. ./clean.sh
2. ./build.sh
3. ./run.sh
4. Run the [simulator](https://github.com/udacity/self-driving-car-sim/releases)

tricks: [Running Simulator on windows and Code on Ubuntu](https://discussions.udacity.com/t/running-simulator-on-windows-and-code-on-ubuntu/255869) or [file](https://github.com/jwangjie/SDC-Monte-Carlo-Localization/blob/master/doc/Running%20Simulator%20on%20windows%20and%20Code%20on%20Ubuntu.pdf)
## Simulation Results
[![demo_gif](https://github.com/jwangjie/SDC-Monte-Carlo-Localization/blob/master/doc/demo.gif)](https://youtu.be/nCiF10BMMfk)

* Inputs:
    * one map contains landmarks
    * initial location based on GPS (x, y) and orientation based on IMU (yaw) at the beginning with uncertainties
    * noisy landmark observations at each timestamp during the car moving

* Outputs: 
    * The real-time pose estimation of the vehicle: the blue circle (estimated location) with a black arrow (estimated orientation) inside 

* Ground truth: 
    * The blue car is the ground truth of the pose of the car
    
## Resources
A good explanation [writing](https://github.com/sohonisaurabh/CarND-Kidnapped-Vehicle-Project) of MCL. 
Note: The sequence of step 4 (Update step) implementation is not proper. Refer to my [code](https://github.com/jwangjie/SDC-Monte-Carlo-Localization/blob/master/src/particle_filter.cpp) comments. 

