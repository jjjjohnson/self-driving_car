# CarND-Path-Planning-Project

This project incorporates sensor fusion and behavior planning to make the car move smoothly along the lane also as fast as possible.  

## Sensor fusion
The sensor fusion data is provided by the JASON object: `auto sensor_fusion = j[1]["sensor_fusion"];`. I iterate through the list and extract the position and velcity of each vehicle. Two vectors `move_left_ok` and `move_right_ok` is introduced to check whether can change lane to left/right respectively. A helper function `not_collide` checks whether it is safe to change lane given this particular vehicle. Only if all the element in `move_left_ok` and `move_right_ok` are ture, can we decide the lane shift is safe.

```
// check if all the element in vector are true
left_allowed = (lane > 0) && all_of(move_left_ok.begin(), move_left_ok.end(), [](bool v) { return v; });

right_allowed = (lane < 2) && all_of(move_right_ok.begin(), move_right_ok.end(), [](bool v) { return v; });
```

## Behavior planning
In order to make sure the vehicle accelerate or decelerate smoothly, I set `ref_vel += 0.1` or `ref_vel -= 0.1` so that when it shifts from decelerate to accelerate the total `ref_vel ` change is 0.2, which does not exceed the acceletion limit. `shift_count` is introduced to make sure when the vehicle start changing lanes, it will finish that lane changing before engaging into another lane change.

Finate state machine design:

- Accelerate.
- Keep lane (follow the vehicle ahead of the ego car).
- Lane change left.
- Lane change right.

Each of the state has a flag to decide whether to active the state or not. The flag is based on the sensor fusion data and the trajectory is calculated using `spline`. Keep lane state recalculates 47 points and only keeps 3 previous points. This is to make the car slow down smoothly as soon as possible before crush into the vehicle ahead of it.