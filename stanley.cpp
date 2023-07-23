#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

// This function calculates the racing line for a given set of spline parameters.
vector<double> racing_line(double s, vector<vector<double>> spline_params) {
  double x = spline_params[0][0] * s * s * s * s + spline_params[0][1] * s * s * s + spline_params[0][2] * s * s + spline_params[0][3] * s + spline_params[0][4];
  double y = spline_params[1][0] * s * s * s * s + spline_params[1][1] * s * s * s + spline_params[1][2] * s * s + spline_params[1][3] * s + spline_params[1][4];
  double heading = atan2(y - spline_params[1][4], x - spline_params[0][4]);
  return {x, y, heading};
}

// This function discretizes the racing line into a list of points.
vector<vector<double>> discretize_parametric(vector<vector<double>> spline_params, double cur_s, double ds) {
  vector<vector<double>> discretized_points;

  double s = cur_s;
  while (s < cur_s + 2 * ds) {
    vector<double> point = racing_line(s);
    discretized_points.push_back(point);
    s += ds;
  }

  return discretized_points;
}

// This function finds the closest waypoint to the vehicle's current position.
vector<double> closest_point(double vehicle_x, double vehicle_y, vector<vector<double>> waypoints) {
  double closest_distance = numeric_limits<double>::max();
  vector<double> closest_wp;

  for (vector<double> wp : waypoints) {
    double dx = wp[0] - vehicle_x;
    double dy = wp[1] - vehicle_y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_wp = wp;
    }
  }

  return closest_wp;
}

double sign(double x) {
  return x > 0 ? 1 : x < 0 ? -1 : 0;
}

// This function implements the Stanley Controller.
// Returns the updated steering angle and velocity.
vector<double> stanley_controller(double vehicle_x, double vehicle_y, 
    double vehicle_theta, double vehicle_velocity, double k, double max_speed, 
    double max_steering_angle, double max_steering_rate, double prev_steering, 
    vector<vector<double>> spline_params, double cur_s, double ds, 
    vector<double> velocity_profile) {
  vector<vector<double>> waypoints = discretize_parametric(spline_params, cur_s, ds);

  // Find the closest waypoint and update cur_s
  vector<double> closest_wp = closest_point(vehicle_x, vehicle_y, waypoints);
  cur_s = closest_wp[2];

  // Compute the errors
  double dx = closest_wp[0] - vehicle_x;
  double dy = closest_wp[1] - vehicle_y;
  double cte = sqrt(dx * dx + dy * dy);
  double psi = closest_wp[2] - vehicle_theta;

  if (vehicle_velocity < 1.0) {
    vehicle_velocity = 1.0;  // to prevent division by zero
  }

  // Control law for the Stanley steering controller
  double unclipped_delta = psi + atan2(k * cte, vehicle_velocity);

  // Clip the steering angle to the maximum allowed
  double delta = max(min(unclipped_delta, max_steering_angle), -max_steering_angle);

  // Limit the rate of change of steering angle
  double steering_rate = (delta - prev_steering) / (vehicle_velocity * ds);
  if (abs(steering_rate) > max_steering_rate) {
    delta = prev_steering + sign(steering_rate) * max_steering_rate * vehicle_velocity * ds;
  }

  double desired_velocity;
  
  // If we're at the maximum steering angle, slow down
  if (abs(delta) == max_steering_angle) {
    desired_velocity = 0.5 * max_speed;  // or some other speed lower than max_speed
  } else {
    // Use the velocity profile to determine the desired velocity
    desired_velocity = velocity_profile[cur_s];
  }

  return {delta, desired_velocity};
}