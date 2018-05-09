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

// Help variables
double max_speed = 49.9;
double max_acc = 0.2;
double diff_v = 0;
double min_dist = 50.0;
double min_back_dist = -10.0;
double dist_target = 30.0;
bool is_in_lane_change = false;
int aimed_lane;
int lane;

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

// Euclidean distance
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Index of closest waypoint
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

// Index to next waypoint
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

// Behavior functions
void laneChangeLeft(){
  aimed_lane = lane - 1;
  diff_v = 0;
  is_in_lane_change = true;
  std::cout << "LC left " << std::endl;
}

void laneChangeRight(){
  aimed_lane = lane + 1;
  diff_v = 0;
  is_in_lane_change = true;
  std::cout << "LC right " << std::endl;
}

void accelerate(){
  diff_v = max_acc;
  is_in_lane_change = false;
  aimed_lane = lane;
  std::cout << "Full speed " << std::endl;
}

void keepDistance(){
  diff_v = 0.01;
  is_in_lane_change = false;
  aimed_lane = lane;
  std::cout << "Keep distance " << std::endl;
}

void decelerate(const double dist){
  is_in_lane_change = false;
  diff_v = -max_acc * (min_dist - dist) / min_dist;
  aimed_lane = lane;
  std::cout << "Brake " << diff_v << std::endl;
}

// Main
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

  // Init conditions
  double target_v = 0;
  int init_lane = 1;

  h.onMessage([&target_v, &init_lane, &map_waypoints_x, &map_waypoints_y,
    &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket
    <uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode){
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
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            json msgJson;

          	//------------------------- SENSOR FUSION --------------------------
            // Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // Define lane variables
            vector<bool> blocked_lane = vector<bool>(3, false);
            vector<double> dist_lane = vector<double>(3, 1000);
            vector<double> v_lane = vector<double>(3, 0.0);
            int car_lane = -1;

            // Check ego car's lane
            if(car_d > 0 && car_d < 4){
              lane = 0;
            }
            // Center lane
            else if(car_d > 4 && car_d < 8){
              lane = 1;
            }
            // Right lane
            else if(car_d > 8 && car_d < 12){
              lane = 2;
            }

            // Convert speed to m/s
            double car_speed_ms = car_speed * 0.44704;

            // Loop through list of Sensor Fusion items
            for( int i = 0; i < sensor_fusion.size(); i++ ){

              // Read detected car's frenet coordinates
              const double & s = sensor_fusion[i][5];
              const double & d = sensor_fusion[i][6];

              // If detected car is not in front or close back skip it
              double dist_s = s - car_s;
              if(dist_s < min_back_dist){
                continue;
              }

              // Check in which lane the detected car is
              // Left lane
              if( d > 0 && d < 4 ){
                car_lane = 0;
              }
              // Center lane
              else if( d > 4 && d < 8 ){
                car_lane = 1;
              }
              // Right lane
              else if( d > 8 && d < 12 ){
                car_lane = 2;
              }
              else{
                continue;
              }

              // Read detected car's velocity
              const double & v_x = sensor_fusion[i][3];
              const double & v_y = sensor_fusion[i][4];
              double v = sqrt(v_x * v_x + v_y * v_y);

              // Check if lane is blocked
              if(dist_s < min_dist){

                blocked_lane[car_lane] = true;

                if(dist_s < dist_lane[car_lane]){
                  dist_lane[car_lane] = dist_s;
                  v_lane[car_lane] = v;
                }
              }
            }

            // Print lane information from Sensor Fusion
            std::cout << "Lanes " << blocked_lane[0] << "  " << blocked_lane[1] 
              << "  " << blocked_lane[2] << std::endl;

            //---------------------------- BEHAVIOUR ---------------------------

            // If car is currently in a lane change manoveur
            if(is_in_lane_change){

              // Lane change is over
              if(abs(car_d - (aimed_lane * 4 + 2)) < 0.4){
                is_in_lane_change = false;
                std::cout << "Lange change is over" << std::endl;
              }
              else{
                std::cout << "Within lane change" << std::endl;
              }
            }

            // If car is not in a lane change manoveur find best action
            if(!is_in_lane_change){
              // If car is in center lane
              if(lane == 1){

                // And this lane is blocked
                if(blocked_lane[lane]){

                  // And the other lanes are also blocked
                  if(blocked_lane[lane - 1] && blocked_lane[lane + 1]){

                    // And the car is faster than the front car
                    if(car_speed_ms > v_lane[lane]){
                      decelerate(dist_lane[lane]);
                    }
                    // And the car is not faster than the front car
                    else{
                      keepDistance();
                    }
                  }

                  // And only the left lane is free
                  else if(!blocked_lane[lane - 1] && blocked_lane[lane + 1]){
                    laneChangeLeft();
                  }

                  // And only the right lane is free
                  else if(blocked_lane[lane - 1] && ! blocked_lane[lane + 1]){
                    laneChangeRight();
                  }

                  // And both lanes are free
                  else{
                    laneChangeLeft();
                  }
                }

                // And lane is free
                else{
                  accelerate();
                }
              }

              // If car is in left lane
              else if(lane == 0){

                // And this lane is blocked
                if(blocked_lane[lane]){

                  // And the center lane is also blocked
                  if(blocked_lane[lane + 1]){

                    // And ego car is faster than front car
                    if(car_speed_ms > v_lane[lane]){
                      decelerate(dist_lane[lane]);
                    }
                    // And ego car is not faster than front car
                    else{
                      keepDistance();
                    }
                  }
                  // And center lane is free
                  else{
                    laneChangeRight();
                  }
                }
                // And lane is free
                else{
                  accelerate();
                }
              }
              // If car is in right lane
              else if(lane == 2){

                // And this lane is blocked
                if(blocked_lane[lane]){

                  // And the center lane is also blocked
                  if(blocked_lane[lane - 1]){

                    // And ego car is faster than front car
                    if(car_speed_ms > v_lane[lane]){
                      decelerate(dist_lane[lane]);
                    }
                    // And ego car is not faster than front car
                    else{
                      keepDistance();
                    }
                  }
                  // And center lane is free
                  else{
                    laneChangeLeft();
                  }
                }
                // And lane is free
                else{
                  accelerate();
                }
              }
            }

            //---------------------- TRAJECTORY PLANNING -----------------------

            // Buffer variables
            vector<double> x_points;
            vector<double> y_points;

            // Reference state
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous points have less than two values
            int prev_size = previous_path_x.size();
            if(prev_size < 2){

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              x_points.push_back(prev_car_x);
              x_points.push_back(car_x);

              y_points.push_back(prev_car_y);
              y_points.push_back(car_y);

            }
            else{

              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              x_points.push_back(ref_x_prev);
              x_points.push_back(ref_x);

              y_points.push_back(ref_y_prev);
              y_points.push_back(ref_y);
            }

            // Defining next way points
            vector<double> next_way_pts_0 = getXY(car_s + 50, 2 + 4 * aimed_lane,
              map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_way_pts_1 = getXY(car_s + 60, 2 + 4 * aimed_lane,
              map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_way_pts_2 = getXY(car_s + 90, 2 + 4 * aimed_lane,
              map_waypoints_s, map_waypoints_x, map_waypoints_y);

            x_points.push_back(next_way_pts_0[0]);
            x_points.push_back(next_way_pts_1[0]);
            x_points.push_back(next_way_pts_2[0]);

            y_points.push_back(next_way_pts_0[1]);
            y_points.push_back(next_way_pts_1[1]);
            y_points.push_back(next_way_pts_2[1]);

            // Transform them to local coordinates
            for(int i = 0; i < x_points.size(); i++){

              double shift_x = x_points[i] - ref_x;
              double shift_y = y_points[i] - ref_y;

              x_points[i] = shift_x * cos(0 - ref_yaw) - 
                shift_y * sin(0 - ref_yaw);
              y_points[i] = shift_x * sin(0 - ref_yaw) + 
                shift_y * cos(0 - ref_yaw);
            }

            // Fit spline.
            tk::spline s;
            s.set_points(x_points, y_points);

            // Use path points from previous path for smoothness
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate target 30 m ahead
            double target_x = dist_target;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            double x_add_on = 0;

            // Loop through waypoints
            for(int i = 1; i < 50 - prev_size; i++){

              // Find target speed
              target_v += diff_v;
              if(target_v > max_speed){
                target_v = max_speed;
              }
              else if(target_v < max_acc){
                target_v = max_acc;
              }

              // Get points
              double N = target_dist / (0.02 * target_v / 2.24);
              double x_point = x_add_on + target_x / N;
              double y_point = s(x_point);
              x_add_on = x_point;

              // Add transformed reference points
              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              // Push back xy pair
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

          	// Add path to json
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
