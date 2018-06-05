# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Output
The final video can be observer below
<p align="center"><a href="https://youtu.be/l5u83bPHjMg" target="_blank"><img src="https://img.youtube.com/vi/l5u83bPHjMg/0.jpg" alt="IMAGE ALT TEXT HERE" width="480" height="300" border="1" /></a></p>

## Rubric Points

* **The Code compiles correctly.**- The code compiled successfully. I used `spline` for smoothening of the generated way points and the `spline.h`  is included in `src` folder of the project directory.

* **The car is able to drive at least 4.32 miles without incident.** - The car is able to drive across the highway easily. I tested for the 15 Miles but the video includes first 4.50 Miles.

* **The car drives according to the speed limit.**- There was not an incident in entire 15 Miles stretch. The car drove within the speed limit at a speed of around 49.5 MPH if there at not any traffic conditions. If there is some obstacle in front of the car, then it tries to change the lane if possible and the drive at full speed else it continues to drive at the same speed as of the car in front.

* **Max Acceleration and Jerk are not Exceeded.** - There was not an incident in entire 15 Miles stretch. 

* **Car does not have collisions.**- There was not an incident in entire 15 Miles stretch. 

* **The car stays in its lane, except for the time between changing lanes.**- The car stays inside the lane, we are just responsible for providing the lane number, the control section is handled by the simulator.

* **The car is able to change lanes**- Whenever there is slow moving traffic the car tries to change the lane and it does it successfully if the adjacent lane is free.


## Model Documentation

The simulator sends a number of things like Car's location, velocity, yaw rate, speed, `frenet` coordinates and sensor fusion data.  For fitting the polynomial in the implementation of this project I used `splin` library. We learned about jerk minimization by using polynomial fitting in our classroom but `splin.h` fits polynomial smoothly and it is well-proven library so I preferred to use this.

### Prediction

We receive data like Car's location, yaw rate, `frenet` coordinates, sensor fusion, velocity and speed from the simulator. By using this data we predict the behavior of other objects in our case vehicles in future and we plan behaviour of our car based on the behaviour of the nearby objects.

### Behaviour Planning 


I have implemented behavior planning in following steps:-

1. check 30 m ahead if there is any car present or not.
2. If a car is present 30 m ahead and it is slow moving try to change the lane.
3. For changing lane first check for both the side lanes(if there are two side lanes else only one) for if there is no vehicle in near proximity from a current location of the car(30m ahead and 5-30m behind the car based on velocity).
4. If there is no vehicle in both the lanes compute velocity of the immediate vehicles in front of the car in both the lanes. And choose the fastest moving lane for lane changing.
5. If no option for lane change then try to maintain a velocity of the Car moving in front.


### Trajectory Generation

To compute trajectory we use car speed, the speed of surrounding cars, current lane, intended lane and previous points. For making trajectory smoother we add immediate two points from previous trajectory. If there are no previous points then we use yaw rate and current car coordinates to compute previous points. Then we add 3 points at 30, 60, 90 meters to the trajectory. To make mathematics bit easier we shift all the points car coordinate system and after processing, we convert them back to map coordinate system before passing those to the car.

### Logic used for switching lanes
```
too_close = CheckIfCarIsTooClose(lane, sensor_fusion, previos_size, car_s, false, 0);
if (too_close)
{
  //to keep moving at the same speed that of the car in front
    if (ref_vel / 2.24 > ComputeSpeedOfLen(lane, sensor_fusion, previos_size, car_s))
    {
        ref_vel -= 0.224;
    }
    int right_lane = lane + 1;
    int left_lane = lane - 1;
    double left_lane_speed = 0;
    double right_lane_speed = 0;
    if (left_lane >= 0 && left_lane < 2)
    {
        if (!CheckIfCarIsTooClose(left_lane, sensor_fusion, previos_size, car_s, true, ceil((60 - ref_vel)/2)))
        {
            left_lane_speed = ComputeSpeedOfLen(left_lane, sensor_fusion, previos_size, car_s);
        }
    }
    if (right_lane <= 2 && right_lane > 0)
    {
        if (!CheckIfCarIsTooClose(right_lane, sensor_fusion, previos_size, car_s, true, ceil((60 - ref_vel)/2)))
        {
            right_lane_speed = ComputeSpeedOfLen(right_lane, sensor_fusion, previos_size, car_s);
        }
    }
    if (right_lane_speed != left_lane_speed)
    {
        lane = right_lane_speed > left_lane_speed ? right_lane : left_lane;
    }
    else if (left_lane_speed != 0)
    {
        lane = left_lane;
    }
}
else if (ref_vel < 49.5)
{
    ref_vel += 0.224;
}
```

I tried with modifying above function so that car can change lane if the fast-moving adjacent lane is available at any time. I found the first approach more efficient and better. In the second approach at some places car wobbles between the lanes. We can see the output of the same in the following video

 <p align="center"><a href="https://youtu.be/PyNT9uD57oA" target="_blank"><img src="https://img.youtube.com/vi/PyNT9uD57oA/0.jpg" alt="IMAGE ALT TEXT HERE" width="480" height="300" border="1" /></a></p>