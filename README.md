# CarND-Kidnapped-Vehicle-Project
<p align='center'>
<img src="https://github.com/wiwawo/CarND-Kidnapped-Vehicle-Project/blob/master/particle_filter_anime.gif" width="480" alt="simulator" />
</p>

A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I will implement a 2 dimensional particle filter in C++. The particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases) . Included in this repository are program files that are used to communicate with the simulator. The simulator provides the script the noisy position data, vehicle controls, and noisy observations. the script feeds back the best particle state.

The simulator can also display the best particle's sensed positions, along with the corresponding map ID associations. This can be extremely helpful when making sure transition and association calculations were done correctly.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Algoritm

INPUT: values provided by the simulator to the c++ program.
    
    ["sense_x", "sense_y", "sense_theta", "previous_velocity", "previous_yawrate"] => the measurements that the simulator observs:
    "sense_x", "sense_y" - noisy gps coordinates of the car;
    "sense_theta" - noisy car orientation;
    "previous velocity" - car velocity;
    "previous_yawrate" - car turning speed.

OUTPUT: coordinates of the particles.

    x, y, θ.

PREDICTION: particles' positions in the next moment t1=t1+dt are calculated using following formula:
    
    x=sense_x+previous velocity/previous_yawrate*[sin(sense_theta+previous_yawrate*dt)−sin(sense_theta)]+x_noise;
    y=sense_y+previous velocity/previous_yawrate*[cos(sense_theta)−cos(sense_theta+previous_yawrate*dt)]+y_noise;  
    θ=sense_theta+previous_yawrate*dt+theta_noise.

LANDMARKS: observed landmarks by the car have coordinates xc, yc, where the point (0,0) is the car itself,
but x,y,θ coordinates of the car are in map coordinate system. Therefore transformation of the  
landmarks' coordinates to map coordinates should be done. The following formula was used for this:

    xm = x + cos θ * xc - sin θ * yc,
    ym = y + sin θ * xc + cos θ * yc.

ASSOCIATION: the car has a map with the landmarks. Using Nearest Neighbor Method observations can be
associated with landmarks on the map.

WEIGHT UPDATE: the lower deviation (error) of the observations and the associated landmarks, the higher weight
will be assign to particles.

The following formula were used to update particles' weight.
<p align='center'>
<img src="https://github.com/wiwawo/CarND-Kidnapped-Vehicle-Project/blob/master/update_formula.png" width="480" alt="update_formula" />
</p>

RESAMPLING: every iteration some particles will be deleted. "Survival" probability for particles will depend on their weights.
The bigger weight, the highter probability for a particle to get to the next iteration.

On the animation in the beginning of this page are shown:

  * a blue vehicle;
  * observations (green lines);
  * map objects (circles with crosses);
  * blue circle is predicted position of the vehicle based on observation and the best particle position;
  * blue lines show association of map objects and observations.

 As it can be seen, particle filters localized the car fast and reliably.

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
  * make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  * gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./particle_filter ` or `bash ../run.sh`
