import math

class PID:
    '''
    Simple PID controller with saturation filtering
    '''
    def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0, dt=0.1, MIN=-math.inf, MAX=math.inf):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.total_error = 0
        self.prev_error = 0
        self.MAX = MAX
        self.MIN = MIN

    def step(self, setpoint, measure):
        # Calculate PID control command
        error = setpoint - measure
        self.total_error += error

        P = self.Kp*error
        I = self.Ki*self.total_error *self.dt
        D = self.Kd*(error - self.prev_error)/self.dt

        self.prev_error = error

        step_command = P + I + D
        step_command = self.saturation_filter(step_command)

        return step_command

    def saturation_filter(self, step_command):
        # Add saturation filter to clamp the command values
        saturated_command = max(self.MIN, min(self.MAX, step_command))
        return saturated_command

class PurePursuit:
    '''
    Pure pursuit controller for controlling agent steering
    '''
    def __init__(self, lookahead_distance, forward_velocity, MIN_LOOKAHEAD_ANGLE=1e-3, MAX_LOOKAHEAD_ANGLE=1.57):
        self.lookahead_distance = lookahead_distance
        self.forward_velocity = forward_velocity
        self.MIN_LOOKAHEAD_ANGLE = MIN_LOOKAHEAD_ANGLE
        self.MAX_LOOKAHEAD_ANGLE = MAX_LOOKAHEAD_ANGLE

    def find_lookahead_point(self, agent_position, path):
        # Determine the lookahead point by finding the point with the closest distance to the lookahead distance
        closest_point = path[-1] # Default to the final path's position to stop the agent
        smallest_error = math.inf

        for point in path:
            distance = math.dist(agent_position, point)
            error = distance - self.lookahead_distance
            if error < smallest_error and error > 0:
                smallest_error = error
                closest_point = point

        return closest_point

    def get_velocity_command(self, agent_pose, path):
        # Calculate the angular velocity commands based on pure pursuit controller
        agent_position = [agent_pose['X'], agent_pose['Y']]
        lookahead_point = self.find_lookahead_point(agent_position, path) # Determine closest point based on lookahead distance
        lookahead_angle = math.atan2(lookahead_point[1] - agent_position[1], lookahead_point[0] - agent_position[0]) - agent_pose['yaw'] # Determine the lookahead angle

        forward_velocity = self.forward_velocity
        if abs(lookahead_angle) < self.MIN_LOOKAHEAD_ANGLE: # Set default value if the lookahead_angle is too small or zero
            lookahead_angle = self.MIN_LOOKAHEAD_ANGLE
        elif abs(lookahead_angle) > self.MAX_LOOKAHEAD_ANGLE: # Set forward velocity to zero for pure rotation if angle difference to large
            forward_velocity = 0

        turning_radius = self.lookahead_distance / (2 * math.sin(lookahead_angle)) # Determine the current turning radius, maintains directionality being left or right
        steering_velocity = self.forward_velocity / turning_radius # Determine the angular velocity required to keep this turning radius *Note can be changed for PID control

        return forward_velocity, steering_velocity
