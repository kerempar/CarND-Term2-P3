# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program

## Kerem Par
<kerempar@gmail.com>


---


## Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

[//]: # (Image References)

[image1]: ./output_images/screenshot1.png =550x350 "Screenshot 1"
[image2]: ./output_images/screenshot2.png =550x350 "Screenshot 2"
[image3]: ./output_images/screenshot3.png =550x350 "Screenshot 3"


### Compiling


Code compiles without errors with cmake and make. 
The following change has been made to the original CMakeLists.txt. The line link_directories(/usr/local/Cellar/libuv/1.11.0/lib) was replaced by link_directories(/usr/local/Cellar/libuv/1.15.0/lib) because of the version of libuv installed.

### Accuracy


Algorithm was run against grader code in the simulator. The particle filter localized vehicle position and yaw to within the values specified in the simulator. The maximum errors observed are shown in the following table:  

| State |  Error  | 
| ----- | ------- |
|  x    |  0.114  | 
|  y    |  0.106  | 
|  yaw  |  0.004  | 

### Performance

The particle filter completed execution within the time of 100 seconds.


The following are the sample outputs of the simulator:

![alt text][image2]

![alt text][image1]

### Flow

The implementation follows the Prediction, Update weigths and Resampling steps of particle filter. 

The number of particles was chosen to be 100.

Here is the main protocol that main.cpp uses in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"] 

["sense_y"] 

["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"] 

["sense_observations_y"] 


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

// message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label 

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


The particle filter also uses the map database (`map_data.txt`) stored in the data folder. It includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id


