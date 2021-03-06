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
#include "helper.h"

extern constexpr double pi();
extern double deg2rad(double x);
extern double rad2deg(double x);

// Load up map values for waypoint's x,y,s and d normalized normal vectors
extern vector<double> map_waypoints_x;
extern vector<double> map_waypoints_y;
extern vector<double> map_waypoints_s;
extern vector<double> map_waypoints_dx;
extern vector<double> map_waypoints_dy;

void getWayPoints(vector<double>& next_x_vals, vector<double>& next_y_vals, const vector<checkCar>& check_cars, const egoCar& ego_car, 
	int& lane, int& prev_lane, double& ref_val, int prev_size,
	double end_path_s, double end_path_d) {

	//video solution
	double car_x = ego_car._x;
    double car_y = ego_car._y;
    double car_s = ego_car._s;
    double car_d = ego_car._d;
    double car_yaw = ego_car._yaw;
    double car_speed = ego_car._speed;

    bool too_close = false;

    

    //First prev_size values in next_x_vals/y_vals are previous_path_x/y
    vector<double> previous_path_x;
    vector<double> previous_path_y;
    for (int i = 0; i < prev_size; ++i) {
    	previous_path_x.push_back(next_x_vals[i]);
    	previous_path_y.push_back(next_y_vals[i]);
    }

    if (prev_size > 0) {
      car_s = end_path_s;
    }

    prev_lane = lane;

    bool check_car0 = false;
    bool check_car1 = false;
    bool check_car2 = false;

    bool switch_lane = false;
    bool nolc0 = false;
    bool nolc1 = false;
    bool nolc2 = false;
    /*
    for(int i = 0; i < check_cars.size(); ++i) {
      float d = check_cars[i]._d;
      double vx = check_cars[i]._vx;
      double vy = check_cars[i]._vy;
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = check_cars[i]._s;

      check_car_s += ((double)prev_size*0.02*check_speed);

      if (d < (2+4*lane+2) && d > (2+4*lane-2)) {
        if ((check_car_s > car_s) && ((check_car_s-car_s) < 40) ) {
            too_close = true;
        }
      }


      //check if other cars in lane
      if (d > 0 && d < 4)
        check_car0 = true;
      if (d > 4 && d < 8)
        check_car1 = true;
      if (d > 8 && d < 12)
        check_car2 = true;
    }
    */

    for(int i = 0; i < check_cars.size(); ++i) {
      float d = check_cars[i]._d;
      double vx = check_cars[i]._vx;
      double vy = check_cars[i]._vy;
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = check_cars[i]._s;

      check_car_s += ((double)prev_size*0.02*check_speed);

      if (d < (2+4*prev_lane+2) && d > (2+4*prev_lane-2)) {
        if ((check_car_s > car_s) && ((check_car_s-car_s) < 40) ) {
            too_close = true;
        }
      }

      if (too_close == true) {
      if ((prev_lane == 0) && (d < 8 && d > 4) && !nolc1) {
        check_car1 = true;
        cout << "cc1: check_car_s - car_s = " << check_car_s - car_s << "\n";
        if ((check_car_s-car_s) > 40 || ((car_s - check_car_s) > 30 && (car_speed > check_speed))) {
            //&& abs(car_speed-check_speed) < 10) {
          lane = 1;
          switch_lane = true;
          cout << "changing lane to ..." << lane << "\n";
          //too_close = false;
        }
        else { lane = prev_lane;
          nolc1 = true;
        }
      }
      else if ((prev_lane == 1) && (d < 4 && d > 0) && !nolc0) {
        check_car0 = true;
        cout << "cc0: check_car_s - car_s = " << check_car_s - car_s << "\n";
//        if (abs(check_car_s-car_s) > 40 && abs(car_speed-check_speed) < 10) {
        if ((check_car_s-car_s) > 40  || ((car_s - check_car_s) > 30 && (car_speed > check_speed)) ) {
            //&& abs(car_speed-check_speed) < 10) {
          lane = 0;
          switch_lane = true;
          cout << "changing lane to ..." << lane << "\n";
          //too_close = false;
        }
        else { lane = prev_lane;
               nolc0 = true;
             }
      }
      else if ((prev_lane == 1) && (d < 12 && d > 8) && !nolc2) {
        check_car2 = true;
        cout << "cc2: check_car_s - car_s = " << check_car_s - car_s << "\n";
        if ((check_car_s-car_s) > 40  || ((car_s - check_car_s) > 30 && (car_speed > check_speed))) {
//            && abs(car_speed-check_speed) < 10) {
          lane = 2;
          switch_lane = true;
          cout << "changing lane to ..." << lane << "\n";
          //too_close = false;
        }
        else  { lane = prev_lane;
                 nolc2 = true;
               }
      }
      else if ((prev_lane == 2) && (d < 8 && d > 4) && !nolc1) {
        check_car1 = true;
        cout << "cc1: check_car_s - car_s = " << check_car_s - car_s << "\n";
        if ((check_car_s-car_s) > 40 || ((car_s - check_car_s) > 30 && (car_speed > check_speed))) {
 //           && abs(car_speed-check_speed) < 10) {
          lane = 1;
          switch_lane = true;
          cout << "changing lane to ..." << lane << "\n";
          //too_close = false;
        }
        else { lane = prev_lane;
             nolc1 = true;
           }
      }
     }

      //Try to come back to center lane if no car there
      //if ((lane == 0 || lane == 2) && (d < 8 && d > 4)) {
      //  if (abs(check_car_s-car_s) > 100 && (car_speed > check_speed)) {
      //    lane = 1;
      //  }
      // }
    }
    //No cars in neighboring lane
    if (too_close == true && switch_lane == false) {
      if ((prev_lane == 0) && (check_car1 == false))
        lane = 1;
      else if ((prev_lane == 1) && (check_car0 == false))
        lane = 0;
      else if ((prev_lane == 1) && (check_car2 == false))
        lane = 2;
      else if ((prev_lane == 2) && (check_car1 == false))
        lane = 1;
    }

    //Don't change two lanes at same time
    if (abs(prev_lane - lane) == 2)
      lane = prev_lane;


    cout << "too close = " << too_close << "\n";
    cout << "check_car0 = " << check_car0 << "\n";
    cout << "check_car1 = " << check_car1 << "\n";
    cout << "check_car2 = " << check_car2 << "\n";

    if(too_close)
      ref_val -= 0.224;
    else if(ref_val < 49.5)
      ref_val += 0.224;

    //widely spaced (x,y) waypoints, evenly spaced at 30m
    vector<double> ptsx;
    vector<double> ptsy;

    //reference x,y, yaw states
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    if(prev_size < 2) {
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      //double prev_car_x = car_x - cos(ref_yaw);
      //double prev_car_y = car_y - sin(ref_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
    }
    else {
      ref_x = previous_path_x[prev_size-1];
      ref_y = previous_path_y[prev_size-1];

      double ref_x_prev = previous_path_x[prev_size-2];
      double ref_y_prev = previous_path_y[prev_size-2];
      ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);
      
      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);

    }

//    std::vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),
    std::vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),
      map_waypoints_s,map_waypoints_x,map_waypoints_y);
    std::vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),
      map_waypoints_s,map_waypoints_x,map_waypoints_y);
    std::vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),
      map_waypoints_s,map_waypoints_x,map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    cout << "car_d new = " << 2+4*lane << "\n";

    for (int i = 0; i < ptsx.size(); ++i) {
      double shift_x = ptsx[i]-ref_x;
      double shift_y = ptsy[i]-ref_y;

      ptsx[i] = (shift_x *cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
      ptsy[i] = (shift_x *sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }

    tk::spline s;

   

 
    s.set_points(ptsx,ptsy);

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x) + 
      (target_y)*(target_y));

    double x_add_on = 0;

    for(int i = 0; i < 50-previous_path_x.size(); i++)
    {

      double N = (target_dist/(0.02*ref_val/2.24));
      double x_point = x_add_on + (target_x)/N;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
      y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }
           
	return;	
}