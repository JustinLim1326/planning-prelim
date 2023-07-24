import math

# Function to create a set of points on the racing line, in the current and the next two time steps
def discretizeParametric(racing_line, cur_t, dt, ddt):
    discretized_points = []
    end_t = cur_t + 2*dt
    
    t = cur_t
    # Loop till end_t to get points on the racing line
    while t < end_t:
        # Get a point on the racing line at time t
        point = racing_line(t)
        # Add the point to the list of discretized points
        discretized_points.append(point)
        # Increment the time by ddt for the next point
        t += ddt

    return discretized_points

# Function to calculate the distance of the car's position from the points on the discretized racing line
def calculateDeviation(new_ego, discretized_parametric):
    last_distance = 10000
    distances = []
    # Calculate the Euclidean distance of the car's position from each point
    for point in discretized_parametric:
        distance = math.sqrt((new_ego.x - point.x)**2 + (new_ego.y - point.y)**2)
        # If the calculated distance is greater than the previous distance, return the distance
        if last_distance < distance:
            return distance

# Function to propagate the car's position and heading based on its velocity and steering angle
def forwardPropagate(ego_state, steering_angle, velocity, dt):
    new_ego = ego_state
    # Update the car's x and y position based on its velocity and current heading
    new_ego.x += velocity * math.cos(ego_state.heading) * dt
    new_ego.y += velocity * math.sin(ego_state.heading) * dt
    # Update the car's heading based on its velocity and steering angle
    new_ego.heading += velocity * math.tan(steering_angle) * dt
    new_ego.steering_angle = steering_angle
    return new_ego

# Function to calculate the steering profile using a Model Predictive Control (MPC) approach
def MPC(racing_line, ego_state, max_steering, max_yaw_rate, steering_list, velocity_profile, dt, ddt, horizons, cur_t):
    steering_profile = []
    final_egos = [ego_state]
    
    # Loop through the prediction horizon
    for horizon in range(horizons):
        distances = []
        steering_angles = []
        
        # Get the discretized racing line for the current time step
        discretized_parametric = discretizeParametric(racing_line, cur_t, ddt)
        
        # Loop through all possible steering angles
        for steering_ratio in steering_list:
            # Calculate the current steering angle
            cur_steering = steering_ratio * max_yaw_rate * dt + ego_state.steering_angle
            # Check if the current steering angle is within the maximum allowable steering range
            if cur_steering > - max_steering and cur_steering < max_steering:
                # Propagate the car's state based on the current steering angle and velocity
                new_ego = forwardPropagate(final_egos[-1], cur_steering, velocity_profile[horizon], dt)
                # Calculate the deviation of the car's new position from the discretized racing line
                distances.append(calculateDeviation(new_ego, discretized_parametric))
                # Store the current steering angle
                steering_angles.append(cur_steering)
        
        # Find the steering angle that resulted in the smallest deviation from the racing line
        min_index = distances.index(min(distances))
        # Add the optimal steering angle to the steering profile
        steering_profile.append(steering_angles[min_index])
        
    # Return the calculated steering and velocity profiles
    return steering_profile, velocity_profile
