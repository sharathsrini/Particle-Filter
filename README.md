# Overview
* Particle filter addresses many problem in applied robotics. Assume we might have moving objects that we want to track. Maybe the objects are fighter jets and missiles, or maybe we are tracking people playing cricket in a field. It doesn't really matter. Let's think about the characteristics of this problem.
* Multimodal: We want to track zero, one, or more than one object simultaneously
* Occlusions: One object can hide another, resulting in one measurement for multiple objects.
* Non-linear behaviour: Aircraft are buffeted by winds, balls move in parabolas, and people collide into each other.
* Non-linear measurements: Radar gives us the distance to an object. Converting that to an (x,y,z) coordinate requires a square root, which is nonlinear.
* Non-Gaussian noise: as objects move across a background the computer vision can mistake part of the background for the object.
* Continuous: the object's position and velocity (i.e. the state space) can smoothly vary over time.
* Multivariate: we want to track several attributes, such as position, velocity, turn rates, etc.
* Unknown process model: we may not know the process model of the system


## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

### Formulae
## Motion Model

![alt text](https://cdn-images-1.medium.com/max/720/1*GT4bfVN82qOPpdCtUwkxig.png)


## Observation

![alt_text](https://cdn-images-1.medium.com/max/720/1*uR0dYxOKWHhEPUT6YMGXGg.png)
![alt_text](https://cdn-images-1.medium.com/max/720/1*nB_6uUWjDKC-pGWToRTQDQ.png)

## Simulation

![alt_text](https://cdn-images-1.medium.com/max/720/1*y92MO5zzieuqxH5-uHD0YQ.png)

## Update Weight
!alt_text](https://cdn-images-1.medium.com/max/720/1*C8vytZC_jE0unb2TSKRUGw.png)





# Implementing the Particle Filter
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
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id




