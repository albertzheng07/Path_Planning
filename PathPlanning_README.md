## Path Planning Project Write up

__

The goals / steps of this project are the following:

* Build Path Planning to drive the car autonomously without an incident for at least 4.32 miles.
* The goal is to avoid any incidents of driving over the speed limit, exceedance of
maximum jerk and acceleration of 10 m/s^3 and 10 m/s^2, avoid collisions,
spend no more than 3 seconds outside the lane lines and also be able to change lanes.

__

## Reflection

For the trajectories, I decided to follow the project walk through and design
the path planner with the following code.

First, I always the previous path generated as the starting path point since the car
may or may not have completed its previous path before generating a new trajectory in
the next time step. If a previous path was not generated, we use the current
state of the car to generate an additional previous point using the vehicle's
current heading. This code is show below.

```C
if (prev_path_size < 2)
{
   prev_car_x = ref_x - cos(car_yaw);
   prev_car_y = ref_y - sin(car_yaw);
}
else           
{
  ref_x = previous_path_x[prev_path_size-1]; // end pt
  ref_y = previous_path_y[prev_path_size-1];

  prev_car_x = previous_path_x[prev_path_size-2]; // end pt -1
  prev_car_y = previous_path_y[prev_path_size-2];

  // calculate yaw between pts
  ref_yaw = atan2(ref_y-prev_car_y,ref_x-prev_car_x);
}
```

Then, using the target lane information, I generated the 3 data points at distances ahead in the frenent coordinate frame which means along the tangent path of the desired highway. These 3 points are then converted back into the global coordinates. These points are pushed into the vector of points ahead of the first two points generated from the previous path. Since it's easier to deal with the points in the car's body coordinate frame for the trajectory generation, we rotate the global points into the car's frame.

```C
// generate 3 pts from the frenet coordinate frame ahead of the initial reference way point
vector <vector <double>> nextWaypoint;

int lane_value = 2+4*current_lane;
vector <double> dist_ahead = {45.0,90.0,135.0};

for (int i = 0; i<dist_ahead.size(); i++)
{
  nextWaypoint.push_back(getXY(car_s+dist_ahead[i],lane_value,map_waypoints_s,map_waypoints_x,map_waypoints_y));

  ptsx.push_back(nextWaypoint[i][0]);
  ptsy.push_back(nextWaypoint[i][1]);              
}

// rotate global car pts to body coordinate frame from current x,y, psi
for (int i = 0; i < ptsx.size(); i++)
{
  double delta_x = ptsx[i]-ref_x;
  double delta_y = ptsy[i]-ref_y;

  ptsx[i] = delta_x*cos(ref_yaw)+delta_y*sin(ref_yaw);
  ptsy[i] = -delta_x*sin(ref_yaw)+delta_y*cos(ref_yaw);

}
```

After that, we generate the actual trajectory first by using the previous path points and then generating the spline with the 5 points generated. The spline is created using the 5 points and then the data points are interpolated along that spline for a short horizon which in this case is 60 m for smoothness. This gives us the series of points added on top of the previous path which serves as our new trajectory. The number of points in the trajectory is always 50 because the number of points added is always 50 - minus the number of points from the previous path that were unfinished. We then take those body coordinate frame points and rotate them back to the global frame to provide to the simulation.

```C
// generate spline
tk::spline s;

// set x y points on spline
s.set_points(ptsx,ptsy);

// set the actual x,y for the planner

// push in previous path first
for (int i = 0; i < prev_path_size; i++)
{
  next_x_vals.push_back(previous_path_x[i]);
  next_y_vals.push_back(previous_path_y[i]);              
}

// generate finite horizon at target distance away using spline
double target_x = 60.0;
double target_y = s(target_x);
double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
double x_step_add = 0;
double dt = 0.02;

for (int i = 1; i < 50 - prev_path_size; i++)
{
  double N_steps = target_dist/dt/(current_target_vel*mph2mps);
  double x_point = x_step_add + target_x/N_steps;
  double y_point = s(x_point);

  x_step_add = x_point;

  // new ref points
  double x_ref = x_point;
  double y_ref = y_point;

  // rotate back to global coordinates for path planner
  x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
  y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

  x_point += ref_x;
  y_point += ref_y;

  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```

After setting up the trajectory generator, the behavior planner was implemented using a state machine to decide which trajectories were to be generated and when. The first thing I implemented was finding the closest car in the current, left lane and right lane. This was done by using the localization data to gather the surrounding cars' position and velocities.

```C
// check all surrounding cars
for (int i = 0; i < sensor_fusion.size(); i++)
{
  // check if other car is in my lane
  float d = sensor_fusion[i][6]; // lateral frenet distance of other car from my car

  int half_lane_width = 2;
  int my_lane = half_lane_width+4*current_lane;
  int left_lane = half_lane_width+4*(current_lane-1);
  int right_lane = half_lane_width+4*(current_lane+1);

  double other_car_vx = sensor_fusion[i][3];
  double other_car_vy = sensor_fusion[i][4];
  double other_car_s = sensor_fusion[i][5];
  double car_speed = sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
  // use previous path to project car's distance into the future
  other_car_s += (double)0.02*car_speed*prev_path_size;

  double car_dist = other_car_s-car_s;
  if (d < (my_lane+half_lane_width) && d > (my_lane-half_lane_width) )
  {
    // get the closest car dist and speed
    if (car_dist > 0.0 && car_dist < car_ahead_dist)
    {
      other_car_speed = car_speed;                  
      car_ahead_dist = car_dist;
    }
  }
  // check closest car in left lane
  if (d < (left_lane+half_lane_width) && d > (left_lane-half_lane_width) )
  {
    if ( fabs(car_dist) < left_car_dist)
    {  
      left_car_speed = car_speed;                  
      left_car_dist = car_dist;
    }
  }
  // check closest car in right lane
  if (d < (right_lane+half_lane_width) && d > (right_lane-half_lane_width) )
  {
    if ( fabs(car_dist) < right_car_dist)
    {  
      right_car_speed = car_speed;                  
      right_car_dist = car_dist;
    }
  }              
}
```

Next, I generated the behavior planner which I broke down into 4 states which was CRUISE_CONTROL (essentially velocity hold with no checks on surroundings until a car in front), POSITION_VELOCITY_HOLD (maintain distance from car in front and hold the same velocity), LANE_SHIFT_LEFT and LANE_SHIFT_RIGHT.

In the CRUISE_CONTROL state, the vehicle would simply target a velocity with a limited acceleration command.

In the POSITION_VELOCITY_HOLD, I added simple proportional control that maintained the position and velocity w.r.t the car ahead. If the position and velocity were within close bounds, I determined that the position velocity hold was stable enough. Ideally, there should be a timer or greater assurance that the vehicle was tracking, but the controller worked well enough without it.

There were no constant actions in the lane shifts required since the trajectory planner were inherit the state transition changes as will be discussed later.

```C

// Vehicle State Machine Actions
switch (Vehicle_state) {  
 case (CRUISE_CONTROL):
 {                
    if (current_target_vel < cruise_target_vel) {
      current_target_vel += accel_max;
    }    

 } break;
 case (POSITION_VELOCITY_HOLD):
 {

    /* Position Loop */
    double position_err = Target_Dist-car_ahead_dist;
    double velocity_cmd = -0.1*position_err;
    /* Velocity Loop */
    velocity_cmd += other_car_speed*1.0/mph2mps; /* Relative to car in front */

    double velocity_err = velocity_cmd - current_target_vel;
    double accel_cmd = 0.05*velocity_err;

    /* clip accel cmds */
    if (accel_cmd > accel_max)
    {
       accel_cmd = accel_max;
    }
    else if (accel_cmd < -decel_max)
    {
       accel_cmd = -decel_max;
    }
    current_target_vel += accel_cmd;

    if (current_target_vel > 50) // respect speed limit
    {
      current_target_vel = 50;
    }

    /* check that position velocity mode is stable */
    postion_vel_hold_stable = false;                
    if (position_err < 5 && velocity_err < 5)
    {
        postion_vel_hold_stable = true;
    }

 } break;

 case (LANE_SHIFT_LEFT):
 {                

 } break;
 case (LANE_SHIFT_RIGHT):
 {
 } break;
}
```

For the behavioral monitoring and state transitions,
CRUISE_CONTROL would transition if POSITION_VELOCITY_HOLD if a car was determined too close by a threshold.

During POSITION_VELOCITY_HOLD, if the vehicle was determined stable, then it would check whether the left lane was clear based on the left lane car's distance and relative speed. If there was sufficient distance and the car was going faster than the other car, then a lane change was initiated. The right lane was done as a secondary option if the left lane wasn't available first although this could be done more elegantly with a cost evaluation.

During the lane change states, the vehicle would transition back to POSITION_VELOCITY_HOLD if a car was too close or if there was sufficient distance, go back to CRUISE_CONTROL.

```C
// State machine transitions
 switch (Vehicle_state) {  
  case (CRUISE_CONTROL):
  {
     if (car_ahead_dist > 0 && car_ahead_dist < 60) // check if car is close and ahead of our car
     {
       Vehicle_state = POSITION_VELOCITY_HOLD; // slow down if that occurs
     }
  } break;
  case (POSITION_VELOCITY_HOLD):
  {
     if (car_ahead_dist < 0 || car_ahead_dist > 120)
     {  
       Vehicle_state = CRUISE_CONTROL;
     }             
      // if stable be ready for turn
     if (postion_vel_hold_stable == true)
     {
       if (current_lane > 0 && fabs(left_car_dist) > 50 && current_target_vel-left_car_speed > 20 )
       {
         Vehicle_state =  LANE_SHIFT_LEFT;
         current_lane -= 1; // change left
       }
       else if (current_lane < 2 && fabs(right_car_dist) > 50 && current_target_vel-right_car_speed > 20 ) // right lane available
       {
         Vehicle_state =  LANE_SHIFT_RIGHT;
         current_lane += 1; // change right
       }
     }
  } break;
  case (LANE_SHIFT_LEFT):
  {
     if (car_d > 2+4*current_lane-2 && car_d < 2+4*current_lane+2)
     {      
       if (car_ahead_dist < 0 || car_ahead_dist > 120)
       {  
         Vehicle_state = CRUISE_CONTROL;
       }
     }
     if (car_ahead_dist > 0 && car_ahead_dist < 40) // check if car is close and ahead of our car
     {
       Vehicle_state = POSITION_VELOCITY_HOLD; // slow down if that occurs
     }                                
  } break;
  case (LANE_SHIFT_RIGHT):
  {
     if (car_d > 2+4*current_lane-2 && car_d < 2+4*current_lane+2)
     {      
       if (car_ahead_dist < 0 || car_ahead_dist > 120)
       {  
         Vehicle_state = CRUISE_CONTROL;
       }
     }
     if (car_ahead_dist > 0 && car_ahead_dist < 30) // check if car is close and ahead of our car
     {
       Vehicle_state = POSITION_VELOCITY_HOLD; // slow down if that occurs
     }            
  } break;
 }
```

As shown with the screenshot below, the car was able to safely drive 10 miles without any incidents.
