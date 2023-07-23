import numpy as np

def racing_line(s, spline_params):
    x = spline_params[0][0] * s ** 5 + spline_params[0][1] * s ** 4 + spline_params[0][2] * s ** 3 + spline_params[0][3] * s ** 2 + spline_params[0][4] * s + spline_params[0][5]
    y = spline_params[1][0] * s ** 5 + spline_params[1][1] * s ** 4 + spline_params[1][2] * s ** 3 + spline_params[1][3] * s ** 2 + spline_params[1][4] * s + spline_params[1][5]
    heading = np.arctan2(y - spline_params[1][5], x - spline_params[0][5])
    return [x, y, heading]

def discretize_parametric(spline_params, cur_s, ds):
    discretized_points = []

    s = cur_s
    while s < cur_s + 2 * ds:
        point = racing_line(s)
        discretized_points.append(point)
        s += ds

    return discretized_points

def closest_point(vehicle_x, vehicle_y, waypoints):
    """
    vehicle_x, vehicle_y: current position of the vehicle
    waypoints: list of (x, y, theta) tuples representing the reference path
    """

    # Find the waypoint closest to the vehicle's current position
    closest_distance = float('inf')
    closest_wp = None

    for wp in waypoints:
        dx = wp[0] - vehicle_x
        dy = wp[1] - vehicle_y
        distance = np.hypot(dx, dy)
        if distance < closest_distance:
            closest_distance = distance
            closest_wp = wp

    return closest_wp

def stanley_controller(vehicle_x, vehicle_y, vehicle_theta, vehicle_velocity, k, max_speed, max_steering_angle, max_steering_rate, prev_steering, spline_params, cur_s, ds, velocity_profile):
    """
    Implements the Stanley Controller.
    Returns the updated steering angle and velocity.
    """

    waypoints = discretize_parametric(spline_params, cur_s, ds)

    # Find the closest waypoint and update cur_s
    closest_wp = closest_point(vehicle_x, vehicle_y, waypoints)
    cur_s = closest_wp[2]

    # Compute the errors
    dx = closest_wp[0] - vehicle_x
    dy = closest_wp[1] - vehicle_y
    cte = np.hypot(dx, dy)
    psi = closest_wp[2] - vehicle_theta

    if vehicle_velocity < 1.0:
        vehicle_velocity = 1.0  # to prevent division by zero

    # Control law for the Stanley steering controller
    unclipped_delta = psi + np.arctan(k * cte / vehicle_velocity)

    # Clip the steering angle to the maximum allowed
    delta = np.clip(unclipped_delta, -max_steering_angle, max_steering_angle)

    # Limit the rate of change of steering angle
    steering_rate = (delta - prev_steering) / (vehicle_velocity * ds)
    if abs(steering_rate) > max_steering_rate:
        delta = prev_steering + np.sign(steering_rate) * max_steering_rate * vehicle_velocity * ds

    # If we're at the maximum steering angle, slow down
    if abs(delta) == max_steering_angle:
        desired_velocity = 0.5 * max_speed  # or some other speed lower than max_speed
    else:
        # Use the velocity profile to determine the desired velocity
        desired_velocity = velocity_profile[cur_s]

    return delta, desired_velocity