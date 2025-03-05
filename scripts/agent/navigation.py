import json
import math
import copy

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

from .safety import EmergencyStop
from .planning import Planning
from .controller import PID, PurePursuit

class Navigation:
    '''
    Navigation class handling waypoint creation and navigatio to each waypoint
    '''

    # POSE_SOURCE='SLAM' or 'ODOMETRY'
    def __init__(self, path_to_config, POSE_SOURCE='SLAM'):
        self.pose_source = POSE_SOURCE

        # Load rostopic names
        with open(path_to_config / 'rostopics.json', 'r') as rostopics_file:
            rostopics = json.load(rostopics_file)

        self.pub = rospy.Publisher(rostopics['command_publisher'], Twist, queue_size=10)
        self.sub_SLAM = rospy.Subscriber(rostopics['SLAM'], PoseStamped, self.SLAM_callback, queue_size=1)
        self.sub_odometry = rospy.Subscriber(rostopics['odometry'], Odometry, self.odometry_callback, queue_size=1)

        # Initialize packages
        self.estop = EmergencyStop(path_to_config)
        self.planning = Planning(path_to_config)

        # Setup Agent SLAM and Odometry poses
        self.pose = {
            'X': 0,
            'Y': 0,
            'Z': 0,
            'roll': 0,
            'pitch': 0,
            'yaw': 0
        }
        self.SLAM = self.pose.copy() # Create copy of template
        self.odometry = self.pose.copy() # Create copy of template

        with open(path_to_config / 'controller_setting.json', 'r') as controller_setting_file:
            controller_setting = json.load(controller_setting_file)

        # Initialize angular positional controller
        angle_control_config = controller_setting['angle_control']
        self.angle_control = PID(
            angle_control_config['gains']['Kp'],
            angle_control_config['gains']['Ki'],
            angle_control_config['gains']['Kd'],
            angle_control_config['settings']['dt'],
            angle_control_config['settings']['MIN_CMD'],
            angle_control_config['settings']['MAX_CMD']
        )
        self.angle_stablization_time = angle_control_config['settings']['stabalization_time']
        self.angle_threshold = angle_control_config['settings']['error_threshold']

        # Initialize positional controller
        path_follow_config = controller_setting['path_following_control_PID']
        self.path_follow_control = PID(
            path_follow_config['gains']['Kp'],
            path_follow_config['gains']['Ki'],
            path_follow_config['gains']['Kd'],
            path_follow_config['settings']['dt'],
            path_follow_config['settings']['MIN_CMD'],
            path_follow_config['settings']['MAX_CMD']
        )
        self.position_stablization_time = path_follow_config['settings']['stabalization_time']
        self.position_threshold = path_follow_config['settings']['error_threshold']
        self.forward_velocity = path_follow_config['settings']['forward_velocity']

        # # Initialize path following controller
        # path_follow_config = controller_setting['path_following_control']
        # self.path_follow_control = PurePursuit(
        #     path_follow_config['lookahead_distance'],
        #     path_follow_config['linear_velocity'],
        #     path_follow_config['MIN_LOOKAHEAD_ANGLE'],
        #     path_follow_config['MAX_LOOKAHEAD_ANGLE']
        # )

    def SLAM_callback(self, msg):
        # Get pose information based on Cartographer SLAM system
        data = msg.pose

        try:
            quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] # Pack as quaternion
            euler_angles = R.from_quat(quaternion).as_euler('xyz') # Create rotation object from quaternion and conver to euler angles
            self.SLAM = {
                'X': data.position.x,
                'Y': data.position.y,
                'Z': data.position.z,
                'roll': euler_angles[0],
                'pitch': euler_angles[1],
                'yaw': euler_angles[2]
            }

            if self.pose_source == 'SLAM':
                self.pose = self.SLAM
        except Exception as e:
            rospy.logerr(f"Pose information processing failed: {e}")

    def odometry_callback(self, data):
        # Get agent odometry based on wheel encoders and IMU
        try:
            quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w] # Pack as quaternion
            euler_angles = R.from_quat(quaternion).as_euler('xyz') # Create rotation object from quaternion and conver to euler angles
            self.odometry = {
                'X': data.pose.pose.position.x,
                'Y': data.pose.pose.position.y,
                'Z': data.pose.pose.position.z,
                'roll': euler_angles[0],
                'pitch': euler_angles[1],
                'yaw': euler_angles[2]
            }
            if self.pose_source == 'ODOMETRY':
                self.pose = self.odometry
        except Exception as e:
            rospy.logerr(f"Odometry information processing failed: {e}")

    def set_velocity(self, linear_velocity, angular_velocity):
        # Move the agent based on linear [m/s] and angular velocity [rad/s]
        #if self.estop.is_estop_active: # Stop sending control commands if emergency stop is active
            #return

        # Publish control commands
        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        self.pub.publish(command)

    def __find_lookahead_point(self, path, L):
        for i, point in enumerate(path):
            if math.dist((self.pose['X'], self.pose['Y']), point) >= L:
                return np.array(point), i
        return np.array(path[0]), 0

    def rotate_to(self, setpoint_angle):
        # Use the angle controller to rotate the agent to the specified angle
        previous_time = rospy.Time.now()
        while not rospy.is_shutdown():
            measured_angle = self.pose['yaw']
            error = measured_angle - setpoint_angle # Fine the error in angle from agent and setpoint
            error = (-error + math.pi) % (2 * math.pi) - math.pi
            angular_velocity = self.angle_control.step(error, 0) # Calculate required angular velocity command

            # Add stabalization time to ensure the agent reaches the setpoint for X amount of time
            current_time = rospy.Time.now()
            if abs(error) < self.angle_threshold:
                if (current_time - previous_time).to_sec() > self.angle_stablization_time:
                    # Reset controller parameters after command is complete
                    self.angle_control.total_error = 0
                    self.angle_control.prev_error = 0
                    return
            else:
                previous_time = rospy.Time.now()

            self.set_velocity(0, angular_velocity) # Send angular velocity control command to agent
            #rospy.sleep(1)

    def follow_path(self, path):
        # Rotate to the first point and start following path to the goal
        lookahead_distance, current_target_index = 0.3, 0
        linear_velocity = self.forward_velocity
        point, _ = self.__find_lookahead_point(path, lookahead_distance)
        # start_x, start_y = path[0] # Get initial point
        start_x, start_y = point
        # Find angle between the agent's heading and initial point
        angle = math.atan2(start_y - self.pose['Y'], start_x - self.pose['X']) # - self.pose['yaw'] 
        self.rotate_to(angle) # Rotate towards initial point
        print("Heading to the path!")

        # Calculate the initial error
        agent_position = [self.pose['X'], self.pose['Y']]
        setpoint_angle = math.atan2(point[1] - agent_position[1], point[0] - agent_position[0])
        error = self.pose['yaw'] - setpoint_angle
        error = (-error + math.pi) % (2 * math.pi) - math.pi
        goal_distance = math.dist(agent_position, path[-1])  

        while goal_distance >= self.position_threshold:
            # Execute the action 
            angular_velocity = self.path_follow_control.step(error, 0)
            self.set_velocity(linear_velocity, angular_velocity)    
            # Update the error and goal distance
            agent_position = [self.pose['X'], self.pose['Y']]
            point, i = self.__find_lookahead_point(path, 0.5)
            setpoint_angle = math.atan2(point[1] - agent_position[1], point[0] - agent_position[0])
            error = self.pose['yaw'] - setpoint_angle
            error = (-error + math.pi) % (2 * math.pi) - math.pi   
            goal_distance = math.dist(agent_position, path[-1])
            # Drop the passed points 
            path = path[i:]       

        print("Destination reached!")
        self.set_velocity(0, 0) # Stop the agent

    def navigate_to_goal(self, goal, is_goal_pixel=True):
        # Pathfind the agent to the goal location
        agent_position = [self.pose['X'], self.pose['Y']]
        agent_pixel_position = self.planning.convert_to_pixel_point(agent_position)
        goal_pixel_position = goal
        if not is_goal_pixel:
            goal_pixel_position = self.planning.convert_to_pixel_point(goal)
        path = self.planning.generate_path(agent_pixel_position, goal_pixel_position)
        real_path = self.planning.convert_to_real_path(path)
        self.follow_path(real_path)

if __name__ == "__main__":
    try:
        Navigation()
    except rospy.ROSInterruptException:
        pass
