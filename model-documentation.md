
# CarND-Path-Planning-Project
This file documents the planning model 
  

## Planning model

The crux of the planning model is contained in the getWayPoints function inside pathplanning.cpp file.

We loop over all the cars from the sensor fusion model (loop starting at line #90). 

The steps are as follows:

1. Detect the speed and s and d coordinates of each neighboring car.

2. Detect if the ego car is too close to the car in front, and set a too_close boolean variable.

3. If too_close is true, try to change lane.

4. Depending on which lane the ego-car is in, look for cars in neighboring lanes.

5. If the car in the neighboring lanes is ahead of the ego car by 40 m or if the ego car is ahead of neighboring lane car by 30 m 
AND the speed of ego-car is greater than the other car, it is safe to try to change the lane.

6. All the cars in neighboring lane that we are intending to switch too has to satisfy the requirement in #5 above.

7. Finally, if too_close is true, but there are no cars in neighboring lanes, it is again to switch lane (loop 
beginning line #173).

8. If too_close is true, decrement ref_val by 0.224, and if ref_val is lower than 49.5 miles/hour, increment 
ref_val by 0.224.

