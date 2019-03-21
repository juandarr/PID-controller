# PID controller implementation

## PID controller

The PID controller, which stands for proportional, integral and differential, is a linear controller designed to stabalize a dinamic linear system in time. It is one the most practical and widely used controller in the industry and I have personally designed one to control a magnetic levitation system, as shown [here](https://juandarr.github.io/projects/maglev). One of the most atrative feaures of the PID controller is that it is quite easy to understand it. It is described by basic Calculus and each term by its math statement, tell us what does to the control signal. The following sections will describe each term of the controller. In these descriptions the error is the difference between the offset or desired value, and the actual output of the system: `(1) error(t) = offset - output(t)`. `dt` is the time step or sampling time. 

### Proportional

```Python
(2) P_term(t) = Kp * error(t)
```
The proportional term at time t is proportional to the error at time t. `Kp` is a constant. This term alone results in oscilations since its value will be negative when error(t) is negative and positive when error(t) is positive (assuming that Kp is positive). In the simulation, we want the car to be positioned at a specific angle: this will be the offset/desired value in the error term. The output is the actual angle of the car on the road. From (1) we get the error, then the P_term is given by (2). When the output is bigger than the offset the action control (P_term) will be negative, steering the car in the left direction. Otherwise it'll be negative, steering the car in the right direction. The term adds overshooting to the system output.

### Differential

```Python
(3) D_term(t) = Kd*(error(t) - error(t-1))/dt
```
The differential control action is proportional to the negative of the derivative of the error. This way, when the error is increasing, which results in a positive derivative, the control action will be positive. Otherwise, when the error is decreasing, which results in a negative derivative, the control action will be negative. In words relevant to the simulation case, if the actual angle of the car is less than the offset/desired value, and reducing (moving from right to left) so that the error increases in time, the action control will define a positive value, **steering the car to the right**. The opossite situation occurs when the actual angle is more than the offset/desired value, and increasing (moving from left to right) so that the error decreases in time, the action control will define a negative value, **steering the car to the left**. The overall effect of this term is serve as a compensation to the oscillations coming from the proportional action. This results in a stable steady state but doesn't take any action when the error remains constant, since `error(t)-error(t-1)` will be zero.

### Integral

```Python
(4) I_term(t) = Ki*sum(all_errors)*dt
 ```
The integral term of the PID controller is proportional to the accumulation of errors in time. If the accumulation of errors is positive (meaning the actual angle is less than the desired value most of the time), the control action will be positive, thus the car will steer to the right. Otherwise , when the accumulation of error is negative (meaning the actual angle is more than the desired value most of the time), the integral control action will be negative, this steering the car in the left direction -moving the angle towards the desired angle-. The effect of this portion of the control is that always accounts for the contrinutions of errors in time, so the offset error (when there is a constant difference between the desired output and the actual output) will be minimized since any additional contribution of errors to the accumulator will result in a non-zero control action.

### Putting all together

Therefore, the PID control is the integration of these three actions. 

```Python
(5) PID = Kp * error(t) + Kd * (error(t) - error(t-1))/dt + Ki * sum(all_errors)*dt
```

## Tunning the PID controller

### Manual tuning
An initial approximation to the solution was achieved manually tuning the PID constants. One way to do this is to start with the proportional constant, moving the value until we reach an almost constant oscillation (increase/decrease values like 1 work for this constant). Then we tune the differential constant, with the goal of improving the steady state of the system. We need to reduce the oscillation in time (increase/decrease values like 0.05 work). Finally, the offset error (if exists), meaning that the error remains constant in time, can be reduce slightly changing the Ki constant (very small increases/decreases like 0.001 work). At the end I came up with the following constants: `Kp = 0.15, Kd =  7.0, Ki = 0.003`.

### Automated tuning with twiddle algorithm

I implemented twiddle in C++ as a method in the PID class. Twiddle finds good PID constants by constantly increasing and decreasing a set of step values define for each constant type, starting with `step_P = 1, step_D = 1, step_I = 1`. The PID constant are initialized to `0`. The algorithm then changes each constant value in the positive direction +step, and runs the process `n` steps. If the results were better than before, it will keep the change and will increase the step of that constant type by 1.1. Otherwise, the constant value is change in the negative direction `-2*step` (this includes the previous transition of `+step` and one additional step in the negative direction `-step`). Runs the process, if the results are better than before, increase the step of that constant type by 1.1. If none of the previous tries worked, decrease the step of that constant type by 0.9. Then move to the next constant. This operation repeats over and over again, until the sum of all the steps of constant types is less than a tolerance value. 
After running this algorithm with an `n` total steps of 500, and more than 110 simulations the algorithm offered the following constants: `Kp = 0.369406, Kd =  14.1166, Ki = 0.00453171`. 
The video at the following link shows the final results: [PID controller to control steering angle]().