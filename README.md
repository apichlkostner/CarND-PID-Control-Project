# PID control for self-driving car

## Summary

- Implementation of a PID controller to control velocity and steering angle of a car.
- Connect the controller to a car simulator.
- Check the behavior with different values of the PID parameters.
- Optimize the PID parameters with coordinate ascent to have a minimum mse.

## Problem

The PID controller should control the steering angle so that the car has a safe behavior. The only controller input is the cross track error (CTE).

Since no car model is used there is also no additional feed forward control. So in curves the CTE increases directly with the change of the trajectory.

## PID controller

A PID controller is used to minimize the difference between the setpoint and the current measured value:

error `e(t) = (setpoint - measurement(t))`

To do this the controller can set a control variable (actuator). This is done using three term which depend on the error directly (proportional term), the derivative of the error (derivative term) and the integral of the error (integral term).

`u(t) = K_p * e(t) + K_d * (e(t) - e(t-1)) * delta_t + K_i * sum(e(t') * delta_t)`

In our case the process variable is the position of the car relative to the optimal trajectory. The control variable is the steering angle.

1) Proportional term: if the distance between the setpoint and the measurement is larger, the proportional term also becomes larger. The proportional term sets the steering angle to zero when the target position is reached. Since the steering angle changed before the car's heading the car trajectory will overshoot the target position and the trajectory will oscillate.

2) Derivative term: is proportional to the change of the error. When the error decreases it reduces the control variable and damps the oscillations from the proportional term. When the error increases suddenly it generates a strong response against it. It is sensitive against high frequency noise of the sensors which can generate high derivatives.

3) Integral term: is proportional to the sum of the previous errors. It compensates drift of the system. If K_i is too high it will generate oscillations since the integrated error remains high when the setpoint is reached.

For most applications the P and I terms are most important for a stable control. The derivative term should be relatively small since it's sensitive to noise.

In out case we have a different behavior. There is no drift and the proportional term alone is enough to bring the system to the setpoint. So K_i can be small.
And since the car's trajectory includes curves which are not modelled with a feed forward term we need a high derivative term to follow the target trajectory in curves.

## Solution

### Manual parameter initialization

First a manual parameter optimization was done so that the car drives safely over the complete track.

1) Only the proportional term was used and the parameter was set so that the car drives with a stable oszillation on the straight part of the track.
2) The derivative term was changed so that the oscillations are damped and the car has a straight trajectory on the straight parts of the track.
3) The integral term was changed to compensate any drift.

After that it was necessary to change the parameters till the behavior was also relatively good during the curves. Since the controller has no information about the trajectory no feed forward term can be used and the PID controller has to find the new steering angle in the curves only from the cross track error. So the result is very bad and has many sharp turns and oscillations.

The parameter of the integral term seemed not to have a big change in the controller behavior but was later set to a relatively high value during the automatic parameter search.

Manual result:

Kp | Kd | Ki
---|----|---
0.08 | 0.1 | 0.02

### Parameter search with coordinate ascent

To find better PID parameters a automatic search was done.
Starting with the manual parameters the car was driven in the simulator and the sum of an error measure was calculated. Then it was searched in the direction of one of the coordinates to find better behavior.

The first error function tried was cte^2 which gave too much oscillations. First improvement was cte^4 to give higher penalty to large cte.

But these functions always found values with much oscillations.

The final idea to give a good passenger comfort was:
1) The car must not leave the road
2) The car should follow the target trajectory as good as possible 
3) The car should use small steering angle when possible, even if the target trajectory is not followed exactly
4) The car should change the sign of the steering angle as few times a possible

The error function is then:

If car don't leaves the road:
`a * cte^2 + b * steer_value^2 + c * numSignChg`

with the hyperparameters

 a   | b    | c
-----|------|-----
0.15 | 0.85 | 0.6

and `numSignChg = "number of sign changes of steer_value"`

If the car leaves the road:
`(RUNNINGCOUNTERMAX - runningcounter) * 1e20` which is a measure how long the car followed the road before it left the road (always higher than a complete track without leaving the road)

The hyperparameters were found by first plotting the cte^2 and steer_value^2 values and weight them to be in a similar range. Finally the parameters where changed to give a good looking result in the simulator.

Results:

Error function | Kp | Kd | Ki
---------------|---|----|---
cte^4| 0.216005 | 0.108957 | 0.128885
final error function | 0.15592 | 0.069404 | 0.05

### Implementation

The PID controller calculated `delta_t` with it's own timer. This is necessary since the value is not constant on all systems (operating system, settings of the simulator) and therefore can't be included in K_i and K_d.

## Result

The car drives safely the complete track but is not really comfortable for a human passenger.

![video of result](docu/pidsmall.mp4)

![image of simulation](docu/simulator.png)

---

# Dependencies

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
