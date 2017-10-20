# PID control
In this project, a vehicle is following a replaned trajectory in a simulator. The vehicle's cross track error(cte) is feedbacked from the simulator and is used by a PID controler to calculated steering value [-1, 1] to keep the vehicle in track.

- Twiddle is used to find optimal factors for Kp, Ki and Kd. The logic is:

**Setup**

Define initial value for PID factor, p=[0, 0, 0,] and change value delta_p=[1, 1, 1].
	
**Loop**

1. Accumulate 300 steps cte. If it is the first time in the loop, assign the accululated cte the the best_error so far and set `is_initialised=true`, else compare the `accum_cte` with `best_error`.
2. If `accum_cte` < `best_error`, update  `best_error` with `accum_cte` and set `delta_p[i] *= 1.1`, else check the flag `up=false`.
3. If `up` is Ture, change `up` to `false`, then `p[i] -= 2*delta_p[i]`, else `p[i] += delta_p[i]` and decrease `dp[i]`.
4. Reset counter `steps=0` and reset the simulator.

Filally, I got the working foactor of:

	Kp = 0.2
	Ki = 0.0001
	Kd = 5.5
	

One thine worth mentioning is that the 300 steps is small (about 5 seconds in the simulator) and does not cover the whole road condition. This results in the bias of the controler to be particular good at certain section of the road while very poor at other parts (such as turning).
The simulator has intrinsic noise and it makes twiddle very sensitive to it. Longer time is required for the twiddle to converge and there is room to make twiddle more robust, such as set a threshold of `accum_cte - best_error`.
	