#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

enum carState_t
{
  CRUISE_CONTROL = 0,
  POSITION_VELOCITY_HOLD = 1,  
  PREPARE_FOR_LANE = 2,
  LANE_SHIFT_LEFT = 3,
  LANE_SHIFT_RIGHT = 4,
};

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
 
    const int starting_lane = 1;
    static int current_lane = starting_lane;
 
    // target velocity
    const double cruise_target_vel = 45.0; // mph
    static double current_target_vel = 0.0;
    static carState_t Vehicle_state = CRUISE_CONTROL;
    static carState_t Prev_Vehicle_state = Vehicle_state;
    static double other_car_speed = 100000.0;
    const double mph2mps = 1/2.24;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            int prev_path_size = previous_path_x.size();

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            
            // vector of x,y points in global frame
            vector <double> ptsx;
            vector <double> ptsy;

            // starting reference states
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            double prev_car_x = car_x;
            double prev_car_y = car_y;
          
            // check previous path size (minimum of 2 pts to use previous path otherwise create a new previous pt)
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

            // generate 1st two pts from either previous points or use current point
            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);

            const double accel_max = 0.5; // 0.05 g
            const double decel_max = 2.0; // 0.2 g
            const double min_speed = 15; // min speed

            // cout << "current velocity = " << current_target_vel << endl;

            double car_ahead_dist = 100000;
            double other_car_speed = 100000;
            double left_car_speed = 100000;
            double left_car_dist = 100000;
            double right_car_speed = 100000;
            double right_car_dist = 100000;

            static double last_position_velocity_hold_spd = current_target_vel;


            // check all surrounding cars
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              // check if other car is in my lane
              float d = sensor_fusion[i][6]; // lateral frenet distance of other car from my car
              if (d < (2+4*current_lane+2) && d > (2+4*current_lane-2) )
              {
                double other_car_vx = sensor_fusion[i][3];
                double other_car_vy = sensor_fusion[i][4];
                double other_car_s = sensor_fusion[i][5];
                double car_speed = sqrt(other_car_vx*other_car_vx+other_car_vy*other_car_vy);
                // use previous path to project car's distance into the future
                other_car_s += (double)0.02*car_speed*prev_path_size;
                // get the closest car dist and speed
                if (other_car_s-car_s > 0.0 && (other_car_s-car_s) < car_ahead_dist)
                {
                  other_car_speed = car_speed;                  
                  car_ahead_dist = other_car_s-car_s;
                }
              }

              // cout << "Car " << i << endl;
              // cout << "d = " << d << endl;


              // check closest car in left lane
              if (d < (2+4*(current_lane-1)+2) && d > (2+4*(current_lane-1)-2) )
              {
                double left_lane_car_vx = sensor_fusion[i][3];
                double left_lane_car_vy = sensor_fusion[i][4];
                double left_lane_car_s = sensor_fusion[i][5];
                double car_speed = sqrt(left_lane_car_vx*left_lane_car_vx+left_lane_car_vy*left_lane_car_vy);
                // use previous path to project car's distance into the future
                left_lane_car_s += (double)0.02*car_speed*prev_path_size;

                // cout << "left_lane_car_s =  " << left_lane_car_s << endl;
                // cout << "left_car_dist =  "  << left_car_dist << endl;

                // get the closest car dist and speed
                if ( fabs(left_lane_car_s-car_s) < left_car_dist)
                {  
                  left_car_speed = car_speed;                  
                  left_car_dist = left_lane_car_s-car_s;
                }
              }

              // check closest car in right lane
              if (d < (2+4*(current_lane+1)+2) && d > (2+4*(current_lane+1)-2) )
              {
                double right_lane_car_vx = sensor_fusion[i][3];
                double right_lane_car_vy = sensor_fusion[i][4];
                double right_lane_car_s = sensor_fusion[i][5];
                double car_speed = sqrt(right_lane_car_vx*right_lane_car_vx+right_lane_car_vy*right_lane_car_vy);
                // use previous path to project car's distance into the future
                right_lane_car_s += (double)0.02*car_speed*prev_path_size;

                // cout << "right_lane_car_s =  " << right_lane_car_s << endl;
                // cout << "right_car_dist =  "  << right_car_dist << endl;                
                // get the closest car dist and speed
                if ( fabs(right_lane_car_s-car_s) < right_car_dist)
                {  
                  right_car_speed = car_speed;                  
                  right_car_dist = right_lane_car_s-car_s;
                }
              }              
            }

            // cout << "Final left_car_dist =  "  << left_car_dist << endl;
            // cout << "Final Right Car Dist = " << right_car_dist << endl;

            cout << "Car Dist Ahead = " << car_ahead_dist << endl;
            cout << "Left Car Dist = " << left_car_dist << endl;
            cout << "Right Car Dist = " << right_car_dist << endl;
            cout << "Current target velocity = " << current_target_vel << endl;

            cout << "Vehicle Current State = " << Vehicle_state << endl;

            const double Target_Dist = 40.0;
            static bool postion_vel_hold_stable = false;
            static double lane_change_target_vel = 0;

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
                // if (current_target_vel > min_speed && other_car_speed < current_target_vel)
                // {  
                //   current_target_vel -= max_decel*(car_ahead_dist-60)/60;
                // } // decel to other car speed

                /* Position Loop */
                double position_err = Target_Dist-car_ahead_dist;
                double velocity_cmd = -0.1*position_err;
                /* Velocity Loop */
                velocity_cmd += other_car_speed*1.0/mph2mps; /* Relative to car in front */

                double velocity_err = velocity_cmd - current_target_vel;
                double accel_cmd = 0.05*velocity_err;
                // cout << "position_err = " << position_err << endl;                
                // cout << "velocity_cmd = " << velocity_cmd << endl;
                // cout << "accel_cmd = " << accel_cmd << endl;

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

                // cout << "Final Accel Command = " << accel_cmd << endl;
                // cout << "Update target velocity = " << current_target_vel << endl;
                postion_vel_hold_stable = false;                
                if (position_err < 5 && velocity_err < 5)
                {
                    postion_vel_hold_stable = true;
                }

             } break;
             case (PREPARE_FOR_LANE):
             {
                // if (postion_vel_hold_stable == true)
                // {
                //   lane_change_target_vel = current_target_vel - 10;
                //   if (lane_change_target_vel < min_speed)
                //   {
                //     lane_change_target_vel = min_speed;
                //   }
                //   postion_vel_hold_stable = false;
                // }
                // if (current_target_vel > lane_change_target_vel) {
                //   current_target_vel -= decel_max;
                // } 
                double position_err = Target_Dist-car_ahead_dist;
                double velocity_cmd = -0.1*position_err;
                /* Velocity Loop */
                velocity_cmd += other_car_speed*1.0/mph2mps; /* Relative to car in front */

                double velocity_err = velocity_cmd - current_target_vel;
                double accel_cmd = 0.05*velocity_err;
                // cout << "position_err = " << position_err << endl;                
                // cout << "velocity_cmd = " << velocity_cmd << endl;
                // cout << "accel_cmd = " << accel_cmd << endl;

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

                // cout << "Final Accel Command = " << accel_cmd << endl;
                // cout << "Update target velocity = " << current_target_vel << endl;
                postion_vel_hold_stable = false;                
                if (position_err < 5 && velocity_err < 5)
                {
                    postion_vel_hold_stable = true;
                }

             }            
             case (LANE_SHIFT_LEFT):
             {                
               // command velocity decrease of 10 mps of when shifting lanes 
              // double velocity_err = last_position_velocity_hold_spd - current_target_vel;
              // double accel_cmd = 0.05*velocity_err;
              // if (accel_cmd > accel_max)
              // {
              //    accel_cmd = accel_max;
              // }
              // else if (accel_cmd < -decel_max)
              // {
              //    accel_cmd = -decel_max;
              // }
              // current_target_vel += accel_cmd;
              
             } break;
             case (LANE_SHIFT_RIGHT):
             {
              // double velocity_err = last_position_velocity_hold_spd - current_target_vel;
              // double accel_cmd = 0.05*velocity_err;
              // if (accel_cmd > accel_max)
              // {
              //    accel_cmd = accel_max;
              // }
              // else if (accel_cmd < -decel_max)
              // {
              //    accel_cmd = -decel_max;
              // }              
              // current_target_vel += accel_cmd;
             } break;
            } 
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

                // if (car_ahead_dist > 0 && car_ahead_dist > 200)
                // {  
                //   Vehicle_state = DRIVE_STRAIGHT;
                // }
                // if (car_ahead_dist > 0 && car_ahead_dist < 60)
                // {  
                //   Vehicle_state = SLOW_DOWN;
                // }               
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

               // Check if left lane exists and is free
               // Vehicle_state =  LANE_SHIFT_LEFT
               // Check if right lane exists and is free
               // Vehicle_state =  LANE_SHIFT_LEFT 
               // car_ahead_dist = 100000.0; // reinit
               // other_car_speed = 100000.0;
             } break; 
             case (PREPARE_FOR_LANE):
             {
                if (car_ahead_dist > 0 && car_ahead_dist < 30) // check if car is close and ahead of our car
                {
                  Vehicle_state = POSITION_VELOCITY_HOLD; // slow down if that occurs
                }                
                // if (fabs(lane_change_target_vel - current_target_vel) < 3)
                // {
                  // left lane check and car is enough away
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
                // }

             } break;

             case (LANE_SHIFT_LEFT):
             {
                // if (fabs(lane_change_target_vel- current_target_vel) < 3 && car_d > 2+4*current_lane-2 && car_d < 2+4*current_lane+2)
                // {   
                //   Vehicle_state = CRUISE_CONTROL; 
                // }   
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
             case (LANE_SHIFT_RIGHT):
             {
                if (car_d > 2+4*current_lane-2 && car_d < 2+4*current_lane+2)
                {  
                  if (car_ahead_dist > 0 && car_ahead_dist < 30) // check if car is close and ahead of our car
                  {
                    Vehicle_state = POSITION_VELOCITY_HOLD; // slow down if that occurs
                  }       
                  if (car_ahead_dist < 0 || car_ahead_dist > 120)
                  {  
                    Vehicle_state = CRUISE_CONTROL;
                  }
                }
                if (car_ahead_dist > 0 && car_ahead_dist < 30) // check if car is close and ahead of our car
                {
                  Vehicle_state = POSITION_VELOCITY_HOLD; // slow down if that occurs
                } 
                // if (fabs(lane_change_target_vel- current_target_vel) < 3 && car_d > 2+4*current_lane-2 && car_d < 2+4*current_lane+2)
                // {   
                //   Vehicle_state = CRUISE_CONTROL; 
                // }             
             } break;
            }

            // if (Vehicle_state != Prev_Vehicle_state)
            // {
            //   cout << "Vehicle State Transition to " << Vehicle_state << endl;
            // }  

            Prev_Vehicle_state = Vehicle_state;

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

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
