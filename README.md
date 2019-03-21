# PID controller documentation

# 1. PID controller

The PID controller, which stands for proportional, integral and differential, is a linear controller designed to stabilize a dynamic linear system in time around an offset value. It is one the most practical and widely used controllers in the industry and I have personally designed one in the past to control a magnetic levitation system, as shown [here](https://juandarr.github.io/projects/maglev). One of the most attractive features of the PID controller is that it is quite easy to understand it. It is described by basic Calculus and each term in its mathematical definition tells us what does to the control signal. The following sections will describe each term of the controller. In these descriptions, the error is the difference between the offset or desired value, and the actual output of the system: `(1) error(t) = offset - output(t)`. `dt` is the time step or sampling time. In the simulation case, the `output(t)` is the `angle` in time, which changes in a range from -25 degrees to 25 degrees.

## 1.1 Proportional

```Python
(2) P_term(t) = Kp * error(t)
```
The proportional term at time `t` is proportional to the error at time t. `Kp` is a constant. This term alone results in oscillations since its value will be negative when the `error(t)` is negative and positive when the `error(t)` is positive (assuming that `Kp` is positive). In the simulation, we want the car to be positioned at a specific angle: this will be the offset/desired value in the error term. The output is the actual angle of the car on the road. From (1) we get the error, then the `P_term` is given by (2). When the output is bigger than the offset the action control (P_term) will be negative, steering the car in the left direction. Otherwise, it'll be negative, steering the car in the right direction. The term adds overshooting to the system output.

## 1.2 Differential

```Python
(3) D_term(t) = Kd*(error(t) - error(t-1))/dt
```
The differential control action is proportional to the derivative of the error. This way, when the error is increasing, which results in a positive derivative, the control action will be positive. Otherwise, when the error is decreasing, which results in a negative derivative, the control action will be negative. In words relevant to the simulation case, if the actual angle of the car is reducing (moving in the direction 25 deg -> -25 deg) so that the error increases in time, the action control will define a positive value, **steering the car to the right**. The opposite situation occurs when the actual angle is increasing (moving in the direction -25 deg -> 25 deg) so that the error decreases in time, the action control will define a negative value, **steering the car to the left**. The overall effect of this term serves as a compensation to the oscillations coming from the proportional action. This results in a stable steady state but doesn't take any action when the error remains constant since `error(t)-error(t-1)` will be zero. Here is where the next term comes handy. 

## 1.3 Integral

```Python
(4) I_term(t) = Ki*sum(all_errors)*dt
 ```
The integral term of the PID controller is proportional to the accumulation of errors in time. If the accumulation of errors is positive (meaning the actual angle is less than the desired value most of the time), the control action will be positive, thus the car will steer to the right. Otherwise, when the accumulation of error is negative (meaning the actual angle is more than the desired value most of the time), the integral control action will be negative, this steering the car in the left direction -moving the angle towards the desired angle-. The overall effect of the integral control is to always account for the contributions of errors in time. An offset error (when there is a constant difference between the desired output and the actual output) will be minimized since any additional contributions of errors to the accumulator will result in a non-zero control action, in a direction appropriate to reduce the error.

## 1.4 Wrapping up

Finally, equation (5) presents the famous PID controller, which is the integration of the previous three actions. 

```Python
(5) PID = Kp * error(t) + Kd * (error(t) - error(t-1))/dt + Ki * sum(all_errors)*dt
```

# 2. Tunning the PID controller

## 2.1 Manual tuning
An initial approximation to the solution was achieved by manually tuning the PID constants. One way to do this is to start with the proportional constant, moving the value until we reach an almost constant oscillation (increase/decrease in amounts of 1 work for this constant). Then we tune the differential constant, with the goal of improving the steady state of the system. We need to reduce the oscillation in time (increase/decrease in amounts of 0.05 work). Finally, the offset error (if exists), meaning that the error remains constant in time, can be reduced by slightly changing the Ki constant (very small increases/decreases in amounts of 0.001 work). At the end I came up with the following constants: `Kp = 0.15, Kd =  7.0, Ki = 0.003`. Here is a short video showing the performance of this controller in simulation: [Manual tuning - PID controller to control steering angle](https://youtu.be/aAup6fCuPtk).

## 2.2 Automated tuning with twiddle algorithm

I implemented Twiddle in C++ as a method in the PID class. Twiddle finds good PID constants by constantly increasing or decreasing a set of step values defined for each constant, starting with `step_P = 1, step_D = 1, step_I = 1`. The PID constants are initialized to `0`. The algorithm then changes each constant value in the positive direction `+step`, and runs the process `n` steps. If the results were better than before (in terms of the overall error), it will keep the change and will increase the step of that constant by 1.1. Otherwise, the constant value is changed in the negative direction `-2*step` (this includes the previous transition of `+step` and one additional step in the negative direction `-step`). Next, twiddle runs the process. If the results are better than before, increase the step of that constant by 1.1. If none of the previous tries worked, decrease the step of that constant by 0.9. Then move to the next constant. This operation repeats over and over again until the sum of all the steps of constant types is less than a tolerance value. The constant loop should move in the direction P-D-I for the reasons explained in the section `Manual tuning`.
After running this algorithm with an `n` total steps of 500 and more than 150 simulations, we reached the following constants: `Kp = 0.369406, Kd =  14.1166, Ki = 0.00453171`. 
The following video shows the final results: [Twiddle tuning - PID controller to control steering angle]().

# 3. Instructions 

This project involves the Udacity Self Driving Car  Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45). The simulator serves as a visual output of the logic performed in the repository program, which creates a server that uses the simulation as a front end. 

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems (`install_ubuntu.sh` and `install_mac.sh`). For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

- `./build.sh`
- `./run.sh`
 
 Any executables can be removed with the following script:

 - `./clean.sh`

After running the program, open the siumulator. You should get a `connected` message in the terminal output, meaning the simulator is connected to the server program and is ready to run. 

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

# License

Copyright (c) Udacity and Juan David Rios. All rights reserved.

Licensed under the [MIT](https://github.com/juandarr/PID-controller/blob/master/LICENSE) License.