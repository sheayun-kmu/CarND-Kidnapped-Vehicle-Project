# Project: Kidnapped Vehicle

### Udacity Self-Driving Car Engineer Nanodegree Program

--

The goals of this project are the following:

* Write (in C++) a particle filter that can be used to track a simulated vehicle's location in a given map.
* Integrate the particle filter to a uWebSocket-based server and verify the effectiveness using simulation.

[//]: # (Image References)
[overview]: ./images/overview.png
[local-10]: ./images/local-10.png
[local-100]: ./images/local-100.png
[local-1000]: ./images/local-1000.png
[local-5]: ./images/local-5.png
[remote-100]: ./images/remote-100.png
[remote-1000]: ./images/remote-1000.png

## Rubric Points

The rubric points considered in the project can be found [here](https://review.udacity.com/#!/rubrics/747/view).

## Program Build & Execution

In the project root directory, execute (in a shell) the following sequence of commands.

```
# ./clean.sh
# ./build.sh
# ./run.sh
```

The program is built in the form of an API server that listens to the port 4567, to which a simulator sends requests and processes responses accordingly.

## Particle Filter

A particle filter captures the behavior of a Bayesian filter by randomly filtering particles according to the likelihood of each particle accurately reflecting the robot's (vehicle's) position. The algorithm can be described as follows.

1. Initialization - generate a set of particles.
2. Prediction - change the state of each particle based on the control input.
3. Measurement Update - calculate each particle's weight defined as the probability of matching measured locations of landmarks.
4. Resampling - randomly select the same number of particles but with higher probability 
for those with higher weights (possibly picked multiple times).

The above steps are repeated on a sequence of inputs that provide the vehicle's control and sensor measurements. Through iterations, the same particle selected multiple times in resampling (Step 4) shall behave differently due to random nature in prediction, i.e., by Gaussian noises introduced in the prediction step (Step 2).

![Particle Filter Algorithm Flowchart][overview]

The figure above illustrates the particle filter's operation, which is excerpted from Lesson 5, Unit 3.

## Implementation

The project directory has the following structure.

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

The algorithm described in the previous section is implemented in the source file `particle_filter.cpp`. The methods newly implemented are:

* init()
* prediction()
* updateWeights()
* resample()

where each performs Step 1, 2, 3, and 4 described in the previous section, respectively. Besides, a separate method `dataAssociation()` implements part of the update process.

### Initialization

Generate a number (an important design parameter, mostly decided in an empirical manner) of particles with three values representing the vehicle's position: `x` and `y` coordinates for location, and `theta` for heading. These three variables, constituting the state variable for the vehicle's location, follow a normal (Gaussian) distribution with mean values given by initial measurement and standard deviations already given.

### Prediction

In the prediction step, all three variables described above are updated with control parameters. Each particle is given a tuple of `(v, y_dot)`, where `v` denotes the vehicle's velocity and `y_dot` the yaw rate.

The prediction is based on the bicycle motion model, where `(x_f, y_f, theta_f)` is predicted by the following equations depending on whether the yaw rate is zero:

1. If the yaw rate is zero:

* `x_f = x_0 + v * dt * cos(theta_0)`
* `y_f = y_0 + v * dt * sin(theta_0)`
* `theta_f = theta_0`

2. Otherwise:

* `x_f = x_0 + v / y_dot * (sin(theta_0 + y_dot * dt) - sin(theta_0))`
* `y_f = y_0 + v / y_dot * (cos(theta_0) - cos(theta_0 + y_dot * dt))`
* `theta_f = theta_0 + y_dot * dt`

where `x_0`, `y_0`, and `theta_0` denote the previous coordinates and heading, respectively, while `dt` gives the time interval. In other words, the above equations define the change of the state variable after a time interval of `dt`. After calculating the variables' predicted mean values, random Gaussian noises are added so that the set of particles capture a probabilistic model.

### Measurement Update

We are given a set of measurments that comprises `x` and `y` coordinates of sensed landmarks. In order to update each particle's probability of accurately capturing the vehicle's position, we first transform the coordinates into the map's coordinate system using the particle's location and heading. After this transformation, each landmark measurement is associated with the closest landmark previously given by the map information. For this purpose, a separate method `dataAssociation()` is used, which will be explained shortly.

After the data association (matching measurements to landmark positions), the particle's probability of being at the position where the measurement has been taken is calculated based on a multivariate Gaussian distribution. That is, the probability of measuring each associated landmark at the given coordinates is calculated and multiplied over the set of measurements to produce the combined probability. This probability is defined as the particle's weight, which will be used in the resampling step described below.

### Data Association

As described above, we associate each measurement observation (transformed into the map's global coordinate system) to one of the landmarks whose locations are already known. In this project, a simple nearest neighbor algorithm is used, where a landmark whose Euclidean distance is minimum is selected. For each observation, we iterate over all the possible (within the sensor range) landmarks and calculate the distance, where the one with the smallest distance is selected and associated.

The algorithm's execution time is proportional to both the number of observations and the number of candidate landmarks. In other words, if we denote by `m` the number of observations and by `n` the number of landmarks, the algorithm's complexity is `O(mn)`. If the number of landmarks and the number of observations are large, the high complexity disencourages the algorithm to be employed. In addition, applying a simple nearest neighbor algorithm has its limitations explained in Lesson 5, Unit 11.

However, within the scope of this project, the numbers of landmarks and observations are not very large, making this solution a viable one (verified by the simulation results). Also, the accuracy of localization (defined by RMSE) is within the specification, further assuring that a simple solution can be adopted.

### Resampling

After the weight of each particle is calculated, a new set of `N` particles are selected (sampled) where `N` is the number of initially generated particles. Each particle's chances of being selected is proportional to its weight. A particle with a high weight can probably be selected multiple times, whereas one with a low weight has scarce chances of being selected.

For implementation, C++ library's `std::discrete_distribution` is used. We initialize a random number generator with `(weights.begin(), weights.end())` where `weights` gives the `std::vector` of `double` corresponding to each particle's weight. Every time a random number is generated using this distribution, an integer in the range `[0, n)` is produced where `n` is the number of elements in the `weights` vector, and each integer's probability is proportional to each value in the same vector. This serves our purpose of the particle filter's resampling process.

## Simulation

Simulations were first performed on a local machine running Ubuntu 18.04, and then verified again on the virtual machine environment provided by Udacity.

![Result for Local Simulation (# of Particles: 10)][local-10]

The above figure gives the result from local simulation with the number of particles set to 10. Since the initial set of particles are generated in a way that they are centered around an initial GPS measurement (instead of being spread to a wide range inside the map), a relatively small number of particles were sufficient to track the motion of the simulated vehicle.

Although the error figures reported here are a snapshot at the end of the simulation, we report and analyze the error given in the final figure, since they show a general tendency. Furthermore, errors during simulation could not be recorded. The *x*, *y*, and *yaw* errors are .181, .159, and .006, respectively. In addition, the simulation was completed in a total of 48.86 secs, which meets the performance requirements given by the 
[project rubric](https://review.udacity.com/#!/rubrics/747/view).

![Result for Local Simulation (# of Particles: 100)][local-100]

The second figure illustrates the result of the same local simulation with the number of particles set to 100. Error values are smaller than the previous simulation with 10 particles, where *x*, *y*, and *yaw* errors are .114, .109, and .004, respectively. This smaller errors indicate that providing a number of particles and relying on probabilistically adjusting/sampling them gives the particle filter an intrinsic robust localization capability. Note that the system time indicated in the simulator (48.88 secs) does not increase as we extend the number of particles, which shows that the system's performance was not saturated.

![Result for Local Simulation (# of Particles: 1000)][local-1000]

Next, we ran the simulation using a greater number (1,000) of particles and the above figure summarizes the result. Note that (1) although the error values are further reduced, the gain by exploiting ten times more particles was only marginal, and that (2) still the system time (48.90 secs) does not increase much, indicating that the system on which the simulation was executed could manage to execute the algorithm without any performance impact.

![Result for Local Simulation (# of Particles: 5)][local-5]

This time, we did the simulation using only five particles. The above figure gives the result, which shows that too few particles might not serve the localization purpose. Using the same number of particles can produce inconsistent result, which is natural due to the random nature of the algorithm. The above results, combined, verifies that (1) the number of particles is an important design parameter for particle filters, and that (2) the number of particles should strike a good balance in the tradeoff relationship between localization accuracy and execution time.

![Result for Remote Simulation (# of Particles: 100)][remote-100]

For comparison purposes, we performed the same simulation on the virtual machine environment provided by Udacity as well. The above figure shows the result obtained from the remote simulation using 100 particles. The *x*, *y*, and *yaw* errors are .113, .110, and .004, respectively, which is comparable to the result from the local simulation using the same number of particles. One thing to note here is that the system time spent in the simulation was 81.10 secs, which is significantly longer than the local simulation counterpart (48.88 secs). This is presumably due to the performance gap between the two environments.

![Result for Remote Simulation (# of Particles: 1000)][remote-1000]

Finally, to verify that the project requires that the simulation be finished within 100 seconds (in other words, to make sure that the implemented code indeed met the performance requirement), we did a round of remote simulation with as many as 1,000 particles. The above figure shows the result where the simulator indicates that the implementation of particle filter failed to run the simulation within the given amount of time.
