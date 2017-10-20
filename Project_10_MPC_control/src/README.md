# Model Predictive Coltrol

## General procedure
**Setup:**

- Hypermeters: Cost factor for cross track error(cte), velocity, acceleration, steering angle change, acceleration change ect. Length of the trajectory and duration of each timestep.
- Define vehicle dynamics and actuator limitations along with other constraints.
- Define the cost function.


**Loop:**

- Transform points in global coordinate to vehicle's coordinate then fit them with third order polymorial;
- Predict state after the latency as the initial state to the model predictive controller.
- Call the optimization [solver](https://projects.coin-or.org/Ipopt). Given the initial state, the solver will return the vector of control inputs that minimizes the cost function.
- Apply the first control input to the vehicle and disregard the rest of controls.
## Hyper parameters

- The length of the trajectory: N = 15. There is a trade-off between the timestep length and efficiency. The vehicle should see far enough to adjust control in case of sharp turn while keep calculations as fast as possible. 
- Duration of each timestep: dt = 0.1 s. Assuming the speed is 2 m/s, the fastest distance the vehicle can see is 3 m with 15 steps ahead, which provides enough reaction time to follow the planed line.
- Cost factor:
cte = 4000; epsi = 4000; v = 1; angle = 100; angle = 100; angle_jerk = 50000; accel_jerk = 1000. The choice of cost factor is somewhat subjective but I want to make cte, sterring angle swanging as small as possible so I assigned big cost factor on them.
