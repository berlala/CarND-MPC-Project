# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


### Overview
The goals / steps of this project are the following:  

* Complete MPC controller in C++.
* Handle 100ms latency between actuations commands and actual acutuator actions.
* Tune MPC controller parameters to ensure stable driving

### Final Result  

[Here](https://youtu.be/iE_wM8v7wNs) is the video that demonstrates the vehicle controlled by MPC successfully drives around the track in the simulator.

![mpc](https://github.com/LevinJ/CarND-MPC-Project/blob/master/mpc.png 

### The Vehicle Model  

Vehicle model describes how the vehicle moves, kinematic model is emploed in this project. Specifically, it includes three elements.

State:  

Vehicle state includes, [x,y,psi,v,cte,epsi], which corresponds to [vehicle position x, vehicle position y, vehicle orientation, cross track error, vehicle orientation error]

Actuators:  

Actuators are the external forces that can impact model state. 

It includes, [delta, a], which corresponds to [steering angle, acceleration]  

Update Equations:  

x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt  
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt  
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt  
v_[t+1] = v[t] + a[t] * dt  
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt  
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt  


### Timestep Length and Frequency  

Timestep Length (N) refers to the number of states MPC simulates in a cycle, and frequence (dt) is how much time elapses between simulated actuations. T is equal to N*dt, the simulated duration.
In theory, T should be as large as possible, while dt should be as small as possible. However, in the case of driving a car, T should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

In this project, I find that N = 12, dt= 0.05 to be the optimal value when the velocity is set as 40.

As expected, if we are to increase dt (like 0.1), the vehicle model will be further inaccurate and cause the driving to be unstable.

Increasing N (for example 25) has a negative impact I didn't expect, it causes the driving to be less stable. With hindsight, I think this is because when we increase N, the solver will need to optimize more variables and obtain less optimal results within fixed 0.05 cpu time. As a result, I find that smaller N actually can achieve better driving behavior.  


### Polynomial Fitting and MPC Preprocessing  

A third polynomial is used to fit reference waypoints. And to make it easier to compute epsi, all the wayspoits are converted to vehicle coordinate system.  

### Model Predictive Control with Latency  

It turns out that 100ms latency has significant impact on the result and must be handled properly.

The resolution is quite simple, just incorporate this latency into the MPC system. After receiving states from the simulator, we first predict its state after 100 ms, and then feed this new state into the solver. This makes sure that the actuation commands we issue to the simulator corresponds well with the current state of the vehicle.  

## Basic Build Instructions  


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
