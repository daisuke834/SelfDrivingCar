#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "cost.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

using namespace std;

int destination_lane;
double ref_velocity_MPH;
vector<Vehicle> vehicles;
string ego_state;
Vehicle closest_vehicle_in_path;

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

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
  double closestLen = 100000;  // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

struct NEXT_POS_VALS {
  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

struct CAR_LOCALIZATION_DATA {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  int lane;
} CarLocalizationData;

struct PREVIOUS_PATH_POS {
  vector<double> previous_path_x;
  vector<double> previous_path_y;
} PreviousPathPos;

struct MAP_WAYPOINTS {
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
} MapWaypoints;

// 2. Getting Started
NEXT_POS_VALS getNextPosVals_2_GettingStarted(
    CAR_LOCALIZATION_DATA CarLocalizationData) {
  vector<double> dummy1, dummy2;
  NEXT_POS_VALS NextPosVals = {dummy1, dummy2};
  double dist_inc = 0.5;
  for (int i = 0; i < 50; i++) {
    NextPosVals.next_x_vals.push_back(
        CarLocalizationData.car_x +
        (dist_inc * i) * cos(deg2rad(CarLocalizationData.car_yaw)));
    NextPosVals.next_y_vals.push_back(
        CarLocalizationData.car_y +
        (dist_inc * i) * sin(deg2rad(CarLocalizationData.car_yaw)));
  }
  return NextPosVals;
}
// END 2. Getting Started

// 3. More Complex Paths
NEXT_POS_VALS getNextPosVals_3_MoreComplexPaths(
    CAR_LOCALIZATION_DATA CarLocalizationData,
    PREVIOUS_PATH_POS PreviousPathPos) {
  vector<double> dummy1, dummy2;
  NEXT_POS_VALS NextPosVals = {dummy1, dummy2};
  int path_size = PreviousPathPos.previous_path_x.size();

  for (int i = 0; i < path_size; i++) {
    NextPosVals.next_x_vals.push_back(PreviousPathPos.previous_path_x[i]);
    NextPosVals.next_y_vals.push_back(PreviousPathPos.previous_path_y[i]);
  }

  double pos_x;
  double pos_y;
  double angle;
  if (path_size == 0) {
    pos_x = CarLocalizationData.car_x;
    pos_y = CarLocalizationData.car_y;
    angle = deg2rad(CarLocalizationData.car_yaw);
  } else {
    pos_x = PreviousPathPos.previous_path_x[path_size - 1];
    pos_y = PreviousPathPos.previous_path_y[path_size - 1];

    double pos_x2 = PreviousPathPos.previous_path_x[path_size - 2];
    double pos_y2 = PreviousPathPos.previous_path_y[path_size - 2];
    angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
  }

  double dist_inc = 0.5;
  for (int i = 0; i < 50 - path_size; i++) {
    NextPosVals.next_x_vals.push_back(
        pos_x + (dist_inc)*cos(angle + (i + 1) * (pi() / 100)));
    NextPosVals.next_y_vals.push_back(
        pos_y + (dist_inc)*sin(angle + (i + 1) * (pi() / 100)));
    pos_x += (dist_inc)*cos(angle + (i + 1) * (pi() / 100));
    pos_y += (dist_inc)*sin(angle + (i + 1) * (pi() / 100));
  }
  return NextPosVals;
}
// END 3. More Complex Paths

// 4. Project Q&A - Keep Lane
NEXT_POS_VALS getNextPosVals_4_KeepLane(
    CAR_LOCALIZATION_DATA CarLocalizationData,
    PREVIOUS_PATH_POS PreviousPathPos, MAP_WAYPOINTS MapWaypoints) {
  vector<double> dummy1, dummy2;
  NEXT_POS_VALS NextPosVals = {dummy1, dummy2};
  double dist_inc = 0.5;
  for (int i = 0; i < 50; ++i) {
    double next_s = CarLocalizationData.car_s + (i + 1) * dist_inc;
    double next_d = 6;
    vector<double> xy =
        getXY(next_s, next_d, MapWaypoints.map_waypoints_s,
              MapWaypoints.map_waypoints_x, MapWaypoints.map_waypoints_y);
    NextPosVals.next_x_vals.push_back(xy[0]);
    NextPosVals.next_y_vals.push_back(xy[1]);
  }
  return NextPosVals;
}

// 4. Project Q&A - Spline
NEXT_POS_VALS getNextPosVals_4_Spline(CAR_LOCALIZATION_DATA CarLocalizationData,
                                      PREVIOUS_PATH_POS PreviousPathPos,
                                      MAP_WAYPOINTS MapWaypoints) {
  vector<double> dummy1, dummy2;
  NEXT_POS_VALS NextPosVals = {dummy1, dummy2};

  int prev_size = PreviousPathPos.previous_path_x.size();

  // Create a list of widely spaced (x,y) waypoints.
  // Later we will interpolate these waypoints with a spline.
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = CarLocalizationData.car_x;
  double ref_y = CarLocalizationData.car_y;
  double ref_yaw = deg2rad(CarLocalizationData.car_yaw);

  if (prev_size < 2) {
    // Use two points that make the path tangent to the car
    double prev_car_x =
        CarLocalizationData.car_x - cos(CarLocalizationData.car_yaw);
    double prev_car_y =
        CarLocalizationData.car_y - sin(CarLocalizationData.car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(CarLocalizationData.car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(CarLocalizationData.car_y);
  } else {
    // use the previous path's end point as starting reference.
    ref_x = PreviousPathPos.previous_path_x[prev_size - 1];
    ref_y = PreviousPathPos.previous_path_y[prev_size - 1];

    double ref_x_prev = PreviousPathPos.previous_path_x[prev_size - 2];
    double ref_y_prev = PreviousPathPos.previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference.
  vector<double> next_wp0 =
      getXY(CarLocalizationData.car_s + 30, (2 + 4 * CarLocalizationData.lane),
            MapWaypoints.map_waypoints_s, MapWaypoints.map_waypoints_x,
            MapWaypoints.map_waypoints_y);
  vector<double> next_wp1 =
      getXY(CarLocalizationData.car_s + 60, (2 + 4 * CarLocalizationData.lane),
            MapWaypoints.map_waypoints_s, MapWaypoints.map_waypoints_x,
            MapWaypoints.map_waypoints_y);
  vector<double> next_wp2 =
      getXY(CarLocalizationData.car_s + 90, (2 + 4 * CarLocalizationData.lane),
            MapWaypoints.map_waypoints_s, MapWaypoints.map_waypoints_x,
            MapWaypoints.map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    // Transformation to local car coodinates
    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // Define the actual (x,y) points we will use for the planner
  for (int i = 0; i < prev_size; ++i) {
    NextPosVals.next_x_vals.push_back(PreviousPathPos.previous_path_x[i]);
    NextPosVals.next_y_vals.push_back(PreviousPathPos.previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired
  // reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points.
  for (int i = 1; i <= 50 - prev_size; ++i) {
    double N = (target_dist / (0.02 * ref_velocity_MPH / 2.24));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it eariler
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    NextPosVals.next_x_vals.push_back(x_point);
    NextPosVals.next_y_vals.push_back(y_point);
  }

  return NextPosVals;
}

void storeSensorFusion(vector<vector<double>> sensor_fusion, int prev_size) {
  vehicles.clear();
  closest_vehicle_in_path =
      Vehicle(0, CarLocalizationData.car_s + 1000, 100, 0);
  // find ref_v to use
  for (int i = 0; i < sensor_fusion.size(); ++i) {
    // car is in my lane
    float check_d = sensor_fusion[i][6];
    int check_relative_lane = static_cast<int>(floor(
        (check_d -
         (2.0 + 4.0 * static_cast<double>(CarLocalizationData.lane) - 2.0)) /
        4.0));
    double check_vx = sensor_fusion[i][3];
    double check_vy = sensor_fusion[i][4];
    double check_speed = sqrt(check_vx * check_vx + check_vy * check_vy);
    double check_car_s = sensor_fusion[i][5];
    double distance = check_car_s - CarLocalizationData.car_s;
    check_car_s +=
        ((double)prev_size * 0.02 *
         check_speed);  // if using previous points can project s value out
    Vehicle vehicle = Vehicle(check_relative_lane, check_car_s, check_speed, 0);
    vehicles.push_back(vehicle);

    if (check_relative_lane == 0 && check_car_s < closest_vehicle_in_path.s &&
        distance > 0) {
      closest_vehicle_in_path =
          Vehicle(check_relative_lane, check_car_s, check_speed, 0);
    }
  }
}

vector<string> getSuccessorStates() {
  vector<string> states;
  states.push_back("KL");
  if (ego_state.compare("KL") == 0) {
    if (CarLocalizationData.lane != 0) {
      states.push_back("PLCL");
    }
    if (CarLocalizationData.lane != 2) {
      states.push_back("PLCR");
    }
  } else if (ego_state.compare("PLCL") == 0) {
    if (CarLocalizationData.lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (ego_state.compare("PLCR") == 0) {
    if (CarLocalizationData.lane != 2) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  } else if (ego_state.compare("LCL") == 0) {
    if (CarLocalizationData.lane != 0) {
      states.push_back("LCL");
    }
  } else if (ego_state.compare("LCR") == 0) {
    if (CarLocalizationData.lane != 2) {
      states.push_back("LCR");
    }
  }
  // If ego_state is "LCL" or "LCR", then just return "KL"
  return states;
}

NEXT_POS_VALS getTrajectory() {
  vector<double> dummy1, dummy2;
  NEXT_POS_VALS NextPosVals = {dummy1, dummy2};

  int prev_size = PreviousPathPos.previous_path_x.size();

  // Create a list of widely spaced (x,y) waypoints.
  // Later we will interpolate these waypoints with a spline.
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = CarLocalizationData.car_x;
  double ref_y = CarLocalizationData.car_y;
  double ref_yaw = deg2rad(CarLocalizationData.car_yaw);

  if (prev_size < 2) {
    // Use two points that make the path tangent to the car
    double prev_car_x =
        CarLocalizationData.car_x - cos(CarLocalizationData.car_yaw);
    double prev_car_y =
        CarLocalizationData.car_y - sin(CarLocalizationData.car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(CarLocalizationData.car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(CarLocalizationData.car_y);
  } else {
    // use the previous path's end point as starting reference.
    ref_x = PreviousPathPos.previous_path_x[prev_size - 1];
    ref_y = PreviousPathPos.previous_path_y[prev_size - 1];

    double ref_x_prev = PreviousPathPos.previous_path_x[prev_size - 2];
    double ref_y_prev = PreviousPathPos.previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference.
  vector<double> next_wp0 =
      getXY(CarLocalizationData.car_s + 30, (2 + 4 * destination_lane),
            MapWaypoints.map_waypoints_s, MapWaypoints.map_waypoints_x,
            MapWaypoints.map_waypoints_y);
  vector<double> next_wp1 =
      getXY(CarLocalizationData.car_s + 60, (2 + 4 * destination_lane),
            MapWaypoints.map_waypoints_s, MapWaypoints.map_waypoints_x,
            MapWaypoints.map_waypoints_y);
  vector<double> next_wp2 =
      getXY(CarLocalizationData.car_s + 90, (2 + 4 * destination_lane),
            MapWaypoints.map_waypoints_s, MapWaypoints.map_waypoints_x,
            MapWaypoints.map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    // Transformation to local car coodinates
    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // Define the actual (x,y) points we will use for the planner
  for (int i = 0; i < prev_size; ++i) {
    NextPosVals.next_x_vals.push_back(PreviousPathPos.previous_path_x[i]);
    NextPosVals.next_y_vals.push_back(PreviousPathPos.previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired
  // reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  double x_add_on = 0;

  // Fill up the rest of our path planner after filling it with previous points.
  for (int i = 1; i <= 50 - prev_size; ++i) {
    double N = (target_dist / (0.02 * ref_velocity_MPH / 2.24));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotating it eariler
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    NextPosVals.next_x_vals.push_back(x_point);
    NextPosVals.next_y_vals.push_back(y_point);
  }

  return NextPosVals;
}

NEXT_POS_VALS generateTrajectory(string state) {
  /*
  Given a possible next state, generate the appropriate trajectory to realize
  the next state.
  */
  NEXT_POS_VALS trajectory;
  if (state.compare("CS") == 0) {
    trajectory = getTrajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = getTrajectory();
  } else if (state.compare("LCL") == 0) {
    trajectory = getTrajectory();
  } else if (state.compare("LCR") == 0) {
    trajectory = getTrajectory();
  } else if (state.compare("PLCL") == 0) {
    trajectory = getTrajectory();
  } else if (state.compare("PLCR") == 0) {
    trajectory = getTrajectory();
  }
  return trajectory;
}

float calculateCost(string NextState, NEXT_POS_VALS trajectory) {
  /*
  Sum weighted cost functions to get total cost for trajectory.
  */
  float cost = 0.0;

  int target_lane;
  if (NextState.compare("CS") == 0) {
    target_lane = 0;
  }
  if (NextState.compare("KL") == 0) {
    target_lane = 0;
  } else if (NextState.compare("LCL") == 0) {
    target_lane = -1;
  } else if (NextState.compare("LCR") == 0) {
    target_lane = 1;
  } else if (NextState.compare("PLCL") == 0) {
    target_lane = -1;
  } else if (NextState.compare("PLCR") == 0) {
    target_lane = 1;
  }

  bool crash_risk = false;
  bool too_close = false;
  int number_of_vehicles = vehicles.size();
  for (int index = 0; index < vehicles.size(); ++index) {
    int check_relative_lane = vehicles[index].lane;
    if (check_relative_lane == 0) {
      double check_car_s = vehicles[index].s;
      // check s values greater than mine and sgap
      // Do some logic here, lower reference velocity so we don't crash into
      // the car in front of us, could also flag to try to change lanes.
      // ref_velocity_MPH = 29.5; // mph
      if ((check_car_s > CarLocalizationData.car_s) &&
          ((check_car_s - CarLocalizationData.car_s) < 40)) {
        crash_risk = true;
      }
    }
    if (check_relative_lane == target_lane) {
      double check_car_s = vehicles[index].s;
      if ((check_car_s - CarLocalizationData.car_s) > -13 &&
          ((check_car_s - CarLocalizationData.car_s) < 40)) {
        too_close = true;
      }
    }
  }

  if (destination_lane != CarLocalizationData.lane &&
      (ego_state.compare("LCL") == 0 || ego_state.compare("LCR") == 0)) {
    // cout << "lane changing" <<endl;
    if (NextState.compare("LCL") == 0 || NextState.compare("LCR") == 0) {
      cost = 0.15;
    } else {
      cost = 0.9;
    }
  } else if (crash_risk) {
    // cout << "risk of crash!!" <<endl;
    if (NextState.compare("KL") == 0) {
      cost = 1.0;
    } else if (NextState.compare("LCL") == 0 || NextState.compare("LCR") == 0) {
      if (too_close) {
        cost = 1.0;
      } else {
        cost = 0.2;
      }
    } else if (NextState.compare("PLCL") == 0 ||
               NextState.compare("PLCR") == 0) {
      if (too_close) {
        cost = 0.9;
      } else {
        cost = 0.3;
      }
    }
  } else {
    if (CarLocalizationData.lane == 0 &&
        (!too_close)) {  // if current lane and center lane is empty
      // cout << "current lane and center lane is empty" <<endl;
      if (NextState.compare("LCR") == 0) {
        cost = 0.05;
      } else if (NextState.compare("KL") == 0 &&
                 NextState.compare("PLCR") == 0) {
        cost = 0.1;
      } else {
        cost = 1.0;
      }
    } else if (CarLocalizationData.lane == 2 &&
               (!too_close)) {  // if current lane and center lane is empty
      // cout << "current lane and center lane is empty" <<endl;
      if (NextState.compare("LCL") == 0) {
        cost = 0.05;
      } else if (NextState.compare("KL") == 0 &&
                 NextState.compare("PLCL") == 0) {
        cost = 0.1;
      } else {
        cost = 1.0;
      }
    } else if (NextState.compare("KL") == 0) {  // if only current lane is empty
      // cout << "only current lane is empty" <<endl;
      cost = 0.0;
    } else {
      cost = 0.8;
    }
  }
  return cost;
}

NEXT_POS_VALS planBehavior(CAR_LOCALIZATION_DATA CarLocalizationData) {
  vector<string> NextStates = getSuccessorStates();
  float cost;
  vector<float> costs;
  vector<NEXT_POS_VALS> candidates_of_final_trajectories;
  vector<string> candidates_of_final_states;

  for (vector<string>::iterator NextState = NextStates.begin();
       NextState != NextStates.end(); ++NextState) {
    NEXT_POS_VALS trajectory = generateTrajectory(*NextState);
    if (trajectory.next_x_vals.size() != 0) {
      cost = calculateCost(*NextState, trajectory);
      costs.push_back(cost);
      candidates_of_final_trajectories.push_back(trajectory);
      candidates_of_final_states.push_back(*NextState);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  string final_state = candidates_of_final_states[best_idx];
  if (ego_state.compare("PLCL") == 0 && final_state.compare("LCL") == 0) {
    destination_lane = CarLocalizationData.lane - 1;
  } else if (ego_state.compare("PLCR") == 0 &&
             final_state.compare("LCR") == 0) {
    destination_lane = CarLocalizationData.lane + 1;
  }

  ego_state = final_state;
  double distance_to_cvip =
      closest_vehicle_in_path.s - CarLocalizationData.car_s;
  if (ego_state.compare("KL") == 0 || ego_state.compare("CS") == 0 ||
      ego_state.compare("PLCL") == 0 || ego_state.compare("PLCR") == 0) {
    if (distance_to_cvip < 35.0 && distance_to_cvip > 0) {
      if (ref_velocity_MPH < min((double)closest_vehicle_in_path.v * 2.2369,
                                 (double)ref_velocity_MPH)) {
        ref_velocity_MPH += 0.224;
      } else if (ref_velocity_MPH >
                 min((double)closest_vehicle_in_path.v * 2.2369,
                     (double)ref_velocity_MPH))
        ref_velocity_MPH -= 0.224;
    } else if (ref_velocity_MPH < 49.5) {
      ref_velocity_MPH += 0.224;
    }
    destination_lane = CarLocalizationData.lane;
  } else if ((ego_state.compare("LCL") == 0 || ego_state.compare("LCR") == 0) &&
             distance_to_cvip < 30.0) {
    // if there is a risk to crash when trying to change lane
    if (ref_velocity_MPH > closest_vehicle_in_path.v * 2.2369) {
      ref_velocity_MPH -= 0.224;
    }
  }
  cout << ego_state << "\t" << ref_velocity_MPH << "\t" << *best_cost
       << "\tlane:" << destination_lane << "/" << CarLocalizationData.lane
       << "\tv_ego/v_cvip=" << ref_velocity_MPH << "/"
       << closest_vehicle_in_path.v * 2.2369 << "\tdist=" << distance_to_cvip
       << endl;
  return candidates_of_final_trajectories[best_idx];
}

int main() {
  ref_velocity_MPH = 0.224;
  ego_state = "CS";
  destination_lane = 1;

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

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // auto sdata = string(data).substr(0, length);
    // cout << sdata << endl;
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

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // vector<double> next_x_vals;
          // vector<double> next_y_vals;

          // TODO: define a path made up of (x,y) points that the car will visit
          // sequentially every .02 seconds
          // ********

          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          int ego_lane = static_cast<int>(floor(car_d / 4.0));
          CarLocalizationData = {car_x,   car_y,     car_s,   car_d,
                                 car_yaw, car_speed, ego_lane};
          PreviousPathPos = {previous_path_x, previous_path_y};
          MapWaypoints = {map_waypoints_x, map_waypoints_y, map_waypoints_s,
                          map_waypoints_dx, map_waypoints_dy};

          storeSensorFusion(sensor_fusion, prev_size);
          NEXT_POS_VALS NextPosVals = planBehavior(CarLocalizationData);

          // Generating trajectory
          // NEXT_POS_VALS NextPosVals =
          // getNextPosVals_2_GettingStarted(CarLocalizationData); NEXT_POS_VALS
          // NextPosVals =
          // getNextPosVals_3_MoreComplexPaths(CarLocalizationData,
          // PreviousPathPos); NEXT_POS_VALS NextPosVals =
          // getNextPosVals_4_KeepLane(CarLocalizationData, PreviousPathPos,
          // MapWaypoints); NEXT_POS_VALS NextPosVals =
          // getNextPosVals_4_Spline(CarLocalizationData, PreviousPathPos,
          // MapWaypoints);
          // *******

          msgJson["next_x"] = NextPosVals.next_x_vals;
          msgJson["next_y"] = NextPosVals.next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
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
