from gradDescent import gradDescent, discretizeParametric, calculateCurvature, generateVelocityProfile, calculateTime, checkCollision, getMaxLatAccel
import numpy as np
import matplotlib.pyplot as plt

def test_gradient_descent():
  optimized_spline_params = gradDescent(spline_params, ego, ds, setting, max_vel, learning_rate, epochs, map, wheelbase, planner)

  cur_s = 0
  discretized_racing_line = discretizeParametric(optimized_spline_params, cur_s, ds)

  print("Optimized Spline Parameters:")
  print(optimized_spline_params)
  print("Discretized Racing Line:")
  print(discretized_racing_line)

  curvatures = calculateCurvature(discretized_racing_line)
  print("Curvatures:")
  print(curvatures)

  velocity_profile = generateVelocityProfile(curvatures, getMaxLatAccel(setting), max_vel)
  print("Velocity Profile:")
  print(velocity_profile)

  total_time = calculateTime(waypoints, velocity_profile)
  print("Total Time:")
  print(total_time)

  if checkCollision(discretized_racing_line, map, ds, wheelbase):
      print("Collision Detected!")
  else:
      print("No Collision Detected!")

def generate_discretized_racing_line(num_points):
    """
    Generates an arbitrary discretized racing line for testing purposes.

    Args:
      num_points: The number of points to generate.

    Returns:
      A list of (x, y, heading) tuples representing the discretized racing line.
    """
    x_coords = np.random.uniform(0, 10, size=num_points)
    y_coords = np.random.uniform(0, 10, size=num_points)

    headings = np.random.uniform(0, 2 * np.pi, size=num_points)

    discretized_racing_line = [(x_coords[i], y_coords[i], headings[i]) for i in range(num_points)]

    return discretized_racing_line

def generate_waypoints(num_waypoints):
    """
    Generates an arbitrary set of waypoints for testing purposes.

    Args:
      num_waypoints: The number of waypoints to generate.

    Returns:
      A list of (x, y) tuples representing the generated waypoints.
    """
    x_coords = np.random.uniform(0, 10, size=num_waypoints)
    y_coords = np.random.uniform(0, 10, size=num_waypoints)

    waypoints = [(x_coords[i], y_coords[i]) for i in range(num_waypoints)]

    return waypoints

class VehicleState:
    """
    Represents the state of a vehicle.

    Attributes:
      x: The x-coordinate of the vehicle.
      y: The y-coordinate of the vehicle.
      heading: The heading angle of the vehicle.
    """

    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

if __name__ == "__main__":
    spline_params = np.array([[1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 1]])
    ego = VehicleState(0, 0, 0)
    ds = 0.1 
    setting = 'static'
    max_vel = 20
    learning_rate = 0.01
    epochs = 100
    map = [(1, 0.5)]
    wheelbase = 2
    planner = 'global'

    num_points = 20
    discretized_racing_line = generate_discretized_racing_line(num_points)
    num_waypoints = 5
    waypoints = generate_waypoints(num_waypoints)

    test_gradient_descent()

    # Plot the optimized racing line and waypoints
    racing_line_x = [point[0] for point in discretized_racing_line]
    racing_line_y = [point[1] for point in discretized_racing_line]
    waypoints_x = [point[0] for point in waypoints]
    waypoints_y = [point[1] for point in waypoints]

    plt.figure()
    plt.plot(racing_line_x, racing_line_y, label='Optimized Racing Line')
    plt.scatter(waypoints_x, waypoints_y, color='red', label='Waypoints')
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.legend()
    plt.title('Optimized Racing Line with Waypoints')
    plt.grid(True)
    plt.show()