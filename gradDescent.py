import torch
import numpy as np
import math

def racingLine(s, spline_params):
  """
  Calculates the (x, y, heading) coordinates of the racing line at s.

  Args:
    s: The parameter along the racing line.
    spline_params: The parameters of the spline that represents the racing line.

  Returns:
    The (x, y, heading) coordinates of the racing line at s.
  """

  x = spline_params[0][0] * s ** 5 + spline_params[0][1] * s ** 4 + spline_params[0][2] * s ** 3 + spline_params[0][3] * s ** 2 + spline_params[0][4] * s + spline_params[0][5]
  y = spline_params[1][0] * s ** 5 + spline_params[1][1] * s ** 4 + spline_params[1][2] * s ** 3 + spline_params[1][3] * s ** 2 + spline_params[1][4] * s + spline_params[1][5]
  heading = np.arctan2(y - spline_params[1][5], x - spline_params[0][5])
  return [x, y, heading]

def discretizeParametric(spline_params, cur_s, ds):
  """
  Discretizes the racing line between s and s + ds.

  Args:
    spline_params: The parameters of the spline that represents the racing line.
    cur_s: The current parameter along the racing line.
    ds: The discretization step size.

  Returns:
    A list of (x, y, heading) tuples representing the discretized racing line.
  """

  discretized_points = []

  s = cur_s - 2 * ds
  while s < cur_s + 2 * ds:
    point = racingLine(s, spline_params)
    discretized_points.append(point)
    s += ds

  return discretized_points

def closestPoint(vehicle_x, vehicle_y, waypoints):
  """
  Finds the closest waypoint to the vehicle.

  Args:
    vehicle_x: The x-coordinate of the vehicle.
    vehicle_y: The y-coordinate of the vehicle.
    waypoints: A list of (x, y, heading) tuples representing the waypoints.

  Returns:
    The index of the closest waypoint.
  """

  # Find the waypoint closest to the vehicle's current position
  closest_distance = float('inf')
  closest_wp = None

  for index, wp in enumerate(waypoints):
    dx = wp[0] - vehicle_x
    dy = wp[1] - vehicle_y
    distance = np.hypot(dx, dy)
    if distance < closest_distance:
      closest_distance = distance
      closest_wp = index

  return closest_wp

def getMaxLatAccel(setting):
  """
  Returns the maximum lateral acceleration for the given setting.

  Args:
    setting: The setting for the maximum lateral acceleration.

  Returns:
    The maximum lateral acceleration.
  """

  if setting == 'static':
    return 1.5 * 9.8 # or whatever
  else:
    # TODO: Return the maximum lateral acceleration from online estimation
    pass

def generateVelocityProfile(curvatures, max_accel, max_vel):
  """
  Generates a velocity profile for the given curvatures.

  Args:
    curvatures: A list of curvatures.
    max_accel: The maximum lateral acceleration.
    max_vel: The maximum velocity.

  Returns:
    A list of velocities.
  """

  velocities = []
  for curvature in curvatures:
    velocities.append(np.clip(max_accel / curvature, 0, max_vel))
  return velocities

def calculateTime(waypoints, velocity_profile):
  """
  Calculates the total time to traverse the given waypoints at the given velocities.

  Args:
    waypoints: A list of (x, y, heading) tuples representing the waypoints.
    velocity_profile: A list of velocities.

  Returns:
    The total time to traverse the waypoints.
  """

  total_time = 0
  for i in range(len(waypoints) - 1):
    dx = waypoints[i + 1][0] - waypoints[i][0]
    dy = waypoints[i + 1][1] - waypoints[i][1]
    dist = np.hypot(dx, dy)
    time = dist / velocity_profile[i]
    total_time += time

  return total_time

def calculateCurvature(waypoints):
  """
  Calculates the curvature of the racing line at each waypoint.

  Args:
    waypoints: A list of (x, y, heading) tuples representing the racing line.

  Returns:
    A list of curvatures.
  """

  curvatures = []
  for index in range(len(waypoints) - 1):
    dx1 = waypoints[index + 1][0] - waypoints[index][0]
    dy1 = waypoints[index + 1][1] - waypoints[index][1]
    dx2 = waypoints[index][0] - waypoints[index - 1][0]
    dy2 = waypoints[index][1] - waypoints[index - 1][1]

    curvature = (dy1 * dx2 - dy2 * dx1) / (dx1 ** 2 + dy1 ** 2) ** (3 / 2)
    curvatures.append(curvature)
  
  return curvatures

def checkCollision(waypoints, map, ds, wheelbase):
  """
  Checks if the vehicle will collide with any of the cones in the map.

  Args:
    waypoints: A list of (x, y, heading) tuples representing the waypoints.
    map: A list of (x, y) tuples representing the cones.
    ds: The discretization step size.
    wheelbase: The length of the vehicle's wheelbase.

  Returns:
    True if the vehicle will collide with any of the cones, False otherwise.
  """

  cur_s = 0
  for waypoint in waypoints:
    for cone in map:
      euclidian_distance = math.sqrt((waypoint.x - cone.x) ** 2 + (waypoint.y - cone.y) ** 2)
      if euclidian_distance < wheelbase:
        return True
    cur_s += ds

  return False

def loss_function_close_to_waypoints(waypoints, spline_params, cur_s, ds, L):
  """
  Calculates the loss function for the racing line that tries to stay close to the waypoints.

  Args:
    waypoints: A list of (x, y) tuples representing the waypoints.
    spline_params: The parameters of the spline that represents the racing line.
    cur_s: The current parameter along the racing line.
    ds: The discretization step size.
    L: The maximum deviation from the waypoints.

  Returns:
    The loss value.
  """

  loss = 0
  discretized_racing_line = discretizeParametric(spline_params, cur_s, ds)
  for i in range(len(waypoints) - 1):
    dx = discretized_racing_line[i][0] - waypoints[i][0]
    dy = discretized_racing_line[i][1] - waypoints[i][1]
    dist = np.hypot(dx, dy)
    loss += max(0, dist - L)

  return loss

def loss_function_same_path(waypoints, spline_params, cur_s, ds):
  """
  Calculates the loss function for the racing line that ensures the path is the same.

  Args:
    waypoints: A list of (x, y) tuples representing the waypoints.
    spline_params: The parameters of the spline that represents the racing line.
    cur_s: The current parameter along the racing line.
    ds: The discretization step size.

  Returns:
    The loss value.
  """

  loss = 0
  discretized_racing_line = discretizeParametric(spline_params, cur_s, ds)
  for i in range(len(waypoints) - 1):
    if waypoints[i] != discretized_racing_line[i]:
      loss += 1

  return loss

def gradDescent(spline_params, ego, ds, setting, max_vel, learning_rate, epochs, map, wheelbase, planner):
  """
  Performs gradient descent to optimize the racing line.

  Args:
    spline_params: The parameters of the spline that represents the racing line.
    ego: The state of the vehicle.
    ds: The discretization step size.
    setting: The setting for the maximum lateral acceleration.
    max_vel: The maximum velocity.
    learning_rate: The learning rate for the gradient descent algorithm.
    epochs: The number of epochs to run the gradient descent algorithm.
    map: A list of (x, y) tuples representing the cones.
    wheelbase: The length of the vehicle's wheelbase.

  Returns:
    The optimized parameters of the spline.
  """

  last_spline_params = None
  cur_s = 0
  waypoints = discretizeParametric(spline_params, cur_s, ds)
  discretized_racing_line = {}

  # Find the closest waypoint and update cur_s
  closest_wp = closestPoint(ego.x, ego.y, waypoints)
  cur_s = closest_wp[2]

  curvatures = calculateCurvature(waypoints)

  max_accel = getMaxLatAccel(setting)

  velocity_profile = generateVelocityProfile(curvatures, max_accel, max_vel)
  L = wheelbase
  
  if planner == 'global':
    loss = loss_function_same_path(waypoints, spline_params, cur_s, ds)
  else:
    loss = loss_function_close_to_waypoints(waypoints, spline_params, cur_s, ds, L)

  for epoch in range(epochs):
    # Calculate the gradient of the loss
    gradient = torch.zeros_like(spline_params)
    for i in range(len(waypoints) - 1):
      dx = waypoints[i + 1][0] - waypoints[i][0]
      dy = waypoints[i + 1][1] - waypoints[i][1]
      dist = torch.hypot(dx, dy)
      gradient += (loss - dist / velocity_profile[i]) * dist / velocity_profile[i] ** 2

    # Update the spline parameters
    last_spline_params = spline_params
    spline_params -= learning_rate * gradient

    # Ensure that the first and last waypoints are the same
    if waypoints[0] != waypoints[-1] and planner == 'global':
      spline_params[0] = last_spline_params[0]
      spline_params[1] = last_spline_params[1]

    # Cache the discretized racing line
    if cur_s not in discretized_racing_line:
      discretized_racing_line[cur_s] = discretizeParametric(spline_params, cur_s, ds)
    waypoints = discretized_racing_line[cur_s]

    # Find the closest waypoint and update cur_s
    closest_wp = closestPoint(ego.x, ego.y, waypoints)
    cur_s = closest_wp[2]

    curvatures = calculateCurvature(waypoints)

    max_accel = getMaxLatAccel(setting)

    velocity_profile = generateVelocityProfile(curvatures, max_accel, max_vel)

    loss = 0
    L = wheelbase
    if planner == 'global':
      loss = loss_function_same_path(waypoints, spline_params, cur_s, ds)
    else:
      loss = loss_function_close_to_waypoints(waypoints, spline_params, cur_s, ds, L)

    if checkCollision(waypoints, map, ds, wheelbase):
      spline_params = last_spline_params
      break

  return spline_params