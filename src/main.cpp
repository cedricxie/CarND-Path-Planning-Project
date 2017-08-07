#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "spline.h"
#include "map.h"
#include "trajectories.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//***************************************************//
//User Defined Parameters Definition
//***************************************************//
double mph2ms(double x) {return x*1609.34/3600;}

int dflag = 7;
// Debug level
int dflag_general = 1;
int dflag_fitting = 3;
int dflag_getXY_short = 7;
int dflag_getXY_full = 8;

int path_count=0;
double max_s = 6945.554;

double t_inc = 5;
double t_n = 250;
double c = 1.0;
double car_speed_max = 18.0;
int max_prev_path_size = t_n;

vector<double> s_history;
vector<double> d_history;

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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/2)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}
  if (dflag >= dflag_general) {cout << setw(25) << "getFrenet - prev_wp: " << prev_wp << " " <<"next_wp: " << next_wp << endl;}
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
	//for(int i = 0; i < prev_wp; i++)
	//{
	//	frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	//}
  frenet_s = maps_s[prev_wp];
	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
  if (dflag >= dflag_getXY_full) {cout << setw(25) << "getXY - s:  "<< s << " " << maps_s[prev_wp+1] << " " << (int)(maps_s.size()-1) << endl;}
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
    if (dflag >= dflag_getXY_full) {cout << setw(25) << "getXY - s:  "<< s << " " << maps_s[prev_wp+1] << " " << (int)(maps_s.size()-1) << endl;}
	}

	int wp2 = (prev_wp+1)%maps_x.size();
  if (dflag >= dflag_getXY_short) {cout << setw(25) << "getXY - wp:  "<< prev_wp << " " << wp2 << endl;}

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double x = maps_x[prev_wp]+seg_s*cos(heading);
	double y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading - pi()/2;

  x += d*cos(perp_heading);
  y += d*sin(perp_heading);

	return {x,y};

}

vector<double> getXY_spline(double s, double d, tk::spline &s_x, tk::spline &s_y, tk::spline &s_dx, tk::spline &s_dy)
{
  double x = s_x(s) + d*s_dx(s);
  double y = s_y(s) + d*s_dy(s);

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
  map_read_in(map_waypoints_x, map_waypoints_y, map_waypoints_s,
    map_waypoints_dx, map_waypoints_dy);

  tk::spline s_x, s_y, s_dx, s_dy;
  spline_fitting(s_x, s_y, s_dx, s_dy, map_waypoints_s,
    map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy);

  /*vector<double> map_waypoints_x_tmp;
  vector<double> map_waypoints_y_tmp;
  vector<double> map_waypoints_s_tmp;
  vector<double> map_waypoints_dx_tmp;
  vector<double> map_waypoints_dy_tmp;*/

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,
     &s_x, &s_y, &s_dx, &s_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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
            //car_d = 6;
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            car_speed = mph2ms(car_speed);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            //***************************************************//
            //Parameters Definition
            //***************************************************//
          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double prev_x, prev_y, rev_yaw_rad;
            double prev_s, prev_d;
            double prev_speed, prev_speed2;
            double prev_a;

            int path_size = previous_path_x.size();
            int prev_path_size;

            double car_s_next;
            double car_speed_next;
            double car_a_next;

            double car_a;
            double t_current;

            double car_s_current;
            double car_speed_current;
            double car_a_current;

            vector <double> end;
            vector <double> start;

            vector <double> tmp_status;
            //***************************************************//
            //Import Previous States
            //***************************************************//
            if (path_size > max_prev_path_size){
              prev_path_size = max_prev_path_size;
            }
            else{
              prev_path_size = path_size;
            }

            for(int i = 0; i < prev_path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            //***************************************************//
            //Evaluate Previous State
            //***************************************************//
            cout << setw(25) << "==================================================================" << endl;
            cout << setw(25) << "Evaluate Previous States" << endl;
            cout << setw(25) << "==================================================================" << endl;
            //cout << setw(25) << "all path points in the prev list: " << endl;
            //for (int i = 0; i<path_size; i++) cout << previous_path_x[i] << ' ' << previous_path_y[i] << ' ' << endl;
            if (s_history.size()>2){
              prev_s = s_history[s_history.size()-1];
              prev_speed = (s_history[s_history.size()-1] - s_history[s_history.size()-2])/(t_inc/t_n);
              prev_speed2 = (s_history[s_history.size()-2] - s_history[s_history.size()-3])/(t_inc/t_n);
              prev_a = (prev_speed - prev_speed2)/(t_inc/t_n);
              s_history.erase( s_history.begin(), s_history.begin() + (int)t_n - prev_path_size );
              //cout << setw(25) << "all path points in the s_history list: " << endl;
              //for (int i = 0; i<s_history.size(); i++) cout << s_history[i] << ' ' << endl;
            }
            else{
              prev_s = 1.249392e+02;
              prev_speed = 0.0;
              prev_a = 0.0;
            }

            car_d = 6.164833e+00;
            //***************************************************//
            //Calculate Current States
            //***************************************************//
            if (prev_speed < 0.05){
              t_current = -3.0;
            }
            else if(prev_speed > car_speed_max){
              t_current = 3.0;
            }
            else{
              t_current = - log(car_speed_max/prev_speed - 1.0)/c;
            }

            car_s_current = prev_s + car_speed_max/c*log((exp(c*(t_current+t_inc/t_n))+1)/(exp(c*t_current)+1));
            car_speed_current = car_speed_max * 1.0/(1.0+exp(-c*(t_current+t_inc/t_n)));
            car_a_current = car_speed_max*c*exp(-c*(t_current+t_inc/t_n))/(1.0+exp(-c*(t_current+t_inc/t_n)))/(1.0+exp(-c*(t_current+t_inc/t_n)));

            car_s_next = car_s_current + car_speed_max/c * log((exp(c*(t_current+t_inc/t_n*(t_n+1)))+1)/(exp(c*t_current)+1));
            car_speed_next = car_speed_max * 1.0/(1.0+exp(-c*(t_current+t_inc/t_n*(t_n+1))));
            car_a_next = car_speed_max*c*exp(-c*(t_current+t_inc/t_n*(t_n+1)))/(1.0+exp(-c*(t_current+t_inc/t_n*(t_n+1))))/(1.0+exp(-c*(t_current+t_inc/t_n*(t_n+1))));

            start = {car_s_current , car_speed_current, car_a_current};
            end = {car_s_next, car_speed_next, car_a_next};

            path_count += t_n - prev_path_size;

            cout << scientific;
            cout << left;
            cout << setw(25) << "==================================================================" << endl;
            cout << setw(25) << "pseudo t:  "<< t_current << endl;
            cout << setw(25) << "added path size:  "<< int(t_n) - prev_path_size << endl;
            cout << setw(25) << "total added path size:  "<< path_count << endl;
            cout << setw(25) << "previous status :  "<< prev_s << " " << prev_d << " " << prev_speed << " "  << endl;
            cout << setw(25) << "previous end status :  "<< end_path_s << " " << end_path_d  << endl;
            cout << setw(25) << "current status:  "<< car_s << " " << car_d << " " << car_speed << " " << car_x << " " << car_y << endl;
            cout << setw(25) << "corrected status:  "<< start[0] << " " << start[1] << " " << start[2] << endl;
            cout << setw(25) << "next status: "<< end[0] << " " << end[1] << " " << end[2] << endl;

            vector <double> trajectory_coeffs = JMT(start, end, t_inc);
            //cout << setw(25) << "JML Coefficients: ";
            //for (auto const& k : trajectory_coeffs) std::cout << k << ' ';
            //cout << endl;

            //***************************************************//
            //Output Next Path States
            //***************************************************//
            cout << setw(25) << "==================================================================" << endl;
          	for(int i = 0; i < max_prev_path_size - path_size; i++)
          	{
              //tmp_status = getXY(trj_evl(trajectory_coeffs, t_inc/t_n*i), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              //tmp_status = getXY(trj_evl(trajectory_coeffs, t_inc/t_n*i), car_d, map_waypoints_s_tmp, map_waypoints_x_tmp, map_waypoints_y_tmp);
              tmp_status = getXY_spline(trj_evl(trajectory_coeffs, t_inc/t_n*i), car_d, s_x, s_y, s_dx, s_dy);
              cout << setw(25) << "pushed in s:  "<< trj_evl(trajectory_coeffs, t_inc/t_n*i) << " " << tmp_status[0] << " " << tmp_status[1] << endl;
              //tmp_status = getXY(s_evl(car_s_current, t_current, car_speed_max, c, t_inc/t_n*i), car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              //cout << setw(25) << "pushed in s:  "<< s_evl(car_s_current, t_current, car_speed_max, c, t_inc/t_n*i) << endl;
              s_history.push_back(trj_evl(trajectory_coeffs, t_inc/t_n*i));
              next_x_vals.push_back(tmp_status[0]);
          	  next_y_vals.push_back(tmp_status[1]);
          	}
            cout << setw(25) << "==================================================================" << endl;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;
            //***************************************************//

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
