# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
 
---
 
## Environment and Objective 
The goal of the project is to design a model predictive controller (MPC) that drives a car autonomously in a simulated environment (Udacity self-driving car simulator). The simulator provides us with waypoints along a optimal reference trajectory. The process objective is to follow this reference track as closely as possible. We also get a feed of measurements from the environment, which is used to define the model.
 
## Model
A kinematic bicycle model is used, which means that all dynamical effects such as inertia, friction and torque are not taken in account. 
 
The model is defined by six dependent variables also called constraints: x/y position (x, y), heading(psi), velocity (v), cross track error(cte) and the error of the heading angle (epsi). These constraints are used to design a cost function, which our controller is trying to optimise. 
The MPC is predicting two independent variables also called actuators: steering angle(delta) and throttle(a). These actuators are the controller's output, which is used to drive the car. 
 
After all we are dealing with an optimisation problem. The MPC' job is to find the right value for the actuators, that best minimizes the cost function. 
 
Here are the equations used to predict the dependent variables at each timestep t+1 :
 
```
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
 
`Lf` is a design parameter, which is taken needed to properly model the vehicle's maneuverability. It basically considers the effect of the distance between the center of mass of the vehicle and the front wheels in terms of maneuverability.
 
 
## Timestep Length and Elapsed Duration (N & dt)
 
The cost function is minimized at each time step for N timesteps into the future. We also have to choose the elapsed duration dt per timestep. We only use the actuator values predicted for the first timestep to maneuver the vehicle. At each time step the entire optimisation problem is solved again. This is the case because our predicted trajectory is only an approximation, that has to be constantly reevaluated to match reality. 
N defines the prediction horizon and has to be properly tuned to make our vehicle drive save around the track. A large prediction horizon makes our car drive more smooth, but leads to a tradeoff of making our controler less responsive and also being more computationally expensive.
A large dt results in a better accuracy but is also more computationally expensive and increases latency.
 
Values of `N` and `dt` were chosen empirically to make sure to get the car save around the track. 
The values are `N=10` and `dt=0.2`. Values 100% above and below were tried but gave worse results following the behaviour described above.
 
 
## MPC Preprocessing
For the sake of simplification waypoints are preprocessed. 
 
* First we shift the car reference angle to 90 degrees by subtracting our current position from all the way points. This way our waypoints start at x/y coordinate 0. Here are the equations for the transformation:
 
```
  double shift_x = ptsx[i] - px;
  double shift_y = ptsy[i] - py;
```
is makes it easier to do the polynomial fit because the numerical optimisation works better when the points are close to the origin.
 
* Second we also rotate all the waypoints in a way that our cars heading angle psi is set to 0.
Here the equations for the transformation:
 
```
  ptsx[i] = (shift_x *cos(0-psi)-shift_y *sin(0-psi));
  ptsy[i] = (shift_x *sin(0-psi)+shift_y *cos(0-psi));
```
The rotation also facilitates our polyfit because now we are looking at something similar to a horizontal line. This makes sure the function is defined. Without the rotation we would look at a more vertical and get into trouble because we could have multiple y values for the same x value.
 
 
After these two transformations our reference system has 0 degrees and is also sitting at the origin. 
 
## Latency
As a requirement the model has to be capable of dealing with 100ms latency. Therefore we have to predict our state 100ms into the future before transforming the state and feeding it into the solver. Here is our prediction model:
 
```
  // predict state in 100ms to deal with latency
  double latency = 0.1; 
  px = px + v*cos(psi)*latency;
  py = py + v*sin(psi)*latency;
  psi = psi + v*steer_value/Lf*latency;
  v = v + throttle_value*latency;
```
 
 



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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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
