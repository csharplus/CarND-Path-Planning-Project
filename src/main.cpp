#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdio.h>
#include <time.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

/***********************************************
* Start global variables used by path planning *
************************************************/
// Speed limit 50 mph
double speed_limit = 50.0;

// Safe distance buffer to keep in meters
double safe_distance = 40.0;

// Start in the middle lane by default: 0: left lane, 1: middle lane; 2: right lane
int curr_lane = 1;

// Reference speed to target
double ref_speed = 0.0;

// Speed increment: about 7m/s2, meeting the requirement <10m/s2
double speed_increment = 0.3;

// Time internal for updates in seconds
double time_interval = 0.02;

// Speed converting constant
double mph_to_meter_per_sec = 2.24;

// Planning distance in meters
double planning_distance = 30.0;

// Total way points in planning
int total_way_points = 50;
/***********************************************
*  End global variables used by path planning  *
************************************************/

/***********************************************
* Start helper functions used by path planning *
************************************************/
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}

// Print out the given message to the console
void log_message(string msg) {
  std::cout << "[" << currentDateTime() << "] " << msg << endl;
}

// Check whether the given car_d is in a specific lane
bool is_in_lane(int lane, double car_d) {
  return car_d < (2+4*lane+2) && car_d > (2+4*lane-2);
}
/***********************************************
*  End helper functions used by path planning  *
************************************************/

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          int prev_path_size = previous_path_x.size();

          // Set car_s to the value of end path s;
          if(prev_path_size > 0) {
            car_s = end_path_s;
          }

          bool is_tailing_close = false;
          bool left_lane_open = true;
          bool right_lane_open = true;

          // Iterate through all the cars tracked by sensor fusion
          for(int i=0; i<sensor_fusion.size(); i++){

            double other_car_vx = sensor_fusion[i][3];
            double other_car_vy = sensor_fusion[i][4];
            double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
            double other_car_s = sensor_fusion[i][5];
            float other_car_d = sensor_fusion[i][6];

            // We need to make adjustments for the future s value of the other car,
            // as we use previous points at present,
            other_car_s += double(prev_path_size) * time_interval * other_car_speed;

            // Get the absolute distance between the current car and the other car
            double car_distance = abs(other_car_s - car_s);

            // Check whether current lane is blocked
            if(is_in_lane(curr_lane, other_car_d)
              && (other_car_s > car_s)
              && (car_distance < safe_distance)) {
                is_tailing_close = true;
            }

            // Check whether left lane is open
            int left_lane = curr_lane - 1;
            if(left_lane_open && (left_lane >= 0)
              && is_in_lane(left_lane, other_car_d)
              && (car_distance < safe_distance)){
                left_lane_open = false;
            }

            // Check whether right lane is open
            int right_lane = curr_lane + 1;
            if(right_lane_open && (right_lane <= 2)
               && is_in_lane(right_lane, other_car_d)
               && (car_distance < safe_distance)){
                right_lane_open = false;
            }
          }

          if(is_tailing_close) {
            // There are three options when the current lane is blocked in front:
            // (1) switching to left lane, (2) switching to right lane, or (3) slowing down.
            if (curr_lane > 0 && left_lane_open){
              curr_lane -= 1;
              log_message("Change to left lane.");
            } else if (curr_lane < 2 && right_lane_open){
              curr_lane += 1;
              log_message("Change to right lane.");
            } else{ // slow down
              ref_speed -= speed_increment;
              log_message("Slow down to increase safe distance.");
            }
          }
          else if(ref_speed < speed_limit - speed_increment) {
            // Ramp up the speed
            ref_speed += speed_increment;
          }

          // Declare a list of widely spaced (x,y) way points, to be filled later using splines
          vector<double> wp_x;
          vector<double> wp_y;

          // Declare some reference values
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_x_prev = car_x;
          double ref_y_prev = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_path_size < 2){
            // Previous path has less than two points, use the car location as starting reference
            ref_x_prev = car_x - cos(car_yaw);
            ref_y_prev = car_y - sin(car_yaw);
          }
          else{
            // Use the last point of the previous path as starting reference
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];

            ref_x_prev = previous_path_x[prev_path_size-2];
            ref_y_prev = previous_path_y[prev_path_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
          }

          wp_x.push_back(ref_x_prev);
          wp_x.push_back(ref_x);
          wp_y.push_back(ref_y_prev);
          wp_y.push_back(ref_y);

          double lane_center_d = (2 + 4*curr_lane);

          // Add way points with three evenly-spaced segments with length of planning_distance
          for(int i=1; i<=3; i++) {
            vector<double> next_wp = getXY(car_s + planning_distance * i, lane_center_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            wp_x.push_back(next_wp[0]);
            wp_y.push_back(next_wp[1]);
          }

          for(int i=0; i<wp_x.size(); i++){
            double delta_x = wp_x[i] - ref_x;
            double delta_y = wp_y[i] - ref_y;

            wp_x[i] = (delta_x * cos(0-ref_yaw) - delta_y * sin(0-ref_yaw));
            wp_y[i] = (delta_x * sin(0-ref_yaw) + delta_y * cos(0-ref_yaw));
          }

          // Declare a spline
          tk::spline s;
          s.set_points(wp_x, wp_y);

          // Add previous path points
          for(int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Find out spline points to travel within constraints
          double target_x = planning_distance;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          double steps = target_dist / (time_interval*ref_speed/mph_to_meter_per_sec);
          double x_step = target_x / steps;
          double x_current = 0;

          for (int i = 1; i <= total_way_points - previous_path_x.size(); i++){

            double x_point = x_current + x_step;
            double y_point = s(x_point);

            x_current = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
