import pickle

import cv2
import rospy
import numpy as np

from agent.agent import Agent

def get_pose_tuple(pose_dict) -> tuple:
    return (pose_dict['X'], pose_dict['Y'])

def record_act(agent, last_pose, path):
    pose_tuple = get_pose_tuple(agent.navigation.pose)
    print(pose_tuple)
    agent.navigation.set_velocity(0.2, 0)
    if pose_tuple[0] != last_pose[0] and pose_tuple[1] != last_pose[1]:      
        path.append(pose_tuple)
    return pose_tuple


def eval_path_following(agent, goal_position):
    agent_position = [agent.navigation.pose['X'], agent.navigation.pose['Y']]
    planner = agent.navigation.planning
    agent_pixel_positon = planner.convert_to_pixel_point(agent_position)
    goal_pixel_position = planner.convert_to_pixel_point(goal_position)

    grid_path = planner.generate_path(agent_pixel_positon, goal_pixel_position)

    display_map = planner.map.copy()
    display_map = display_map.astype(np.uint8)
    display_map = cv2.cvtColor(display_map, cv2.COLOR_GRAY2RGB)
    display_map[agent_pixel_positon[0]][agent_pixel_positon[1]] = (255, 0, 0)
    display_map[goal_pixel_position[0]][goal_pixel_position[1]] = (255, 0, 255)

    for x, y in grid_path:
        display_map[y][x] = planner.PATH_COLOUR
    cv2.imwrite("path.png", display_map)

    real_path = planner.convert_to_real_path(grid_path)
    agent.navigation.follow_path(real_path)
    rospy.sleep(0.1)

if __name__ == "__main__":
    record = False

    agent = Agent('SLAM')
    # goal = [3, 1]
    goal = [0, 0]
    # controller = agent.navigation.path_follow_control
    rospy.sleep(1)
    eval_path_following(agent, goal)
    # agent_pose = agent.navigation.pose
    # last_pose = (-1000, -1000)

    # if record:
    #     print("Record mode")
    #     path = []

    #     for i in range(1000):
    #         last_pose = record_act(agent, last_pose, path)
    #         rospy.sleep(0.01)
    #     with open("data.pkl", "wb") as f:
    #         pickle.dump(path, f)
    #         print(path)
    #         print("Record complete!")
    # else:
    #     print("Reading from data")
    #     with open("data.pkl", "rb") as f:
    #         path = pickle.load(f) 

    #     path = path[::-1]
    #     for i in range(100):
    #         eval_act(agent, path, controller)
    #         rospy.sleep(0.1)
    #     print(path)
    #     print("Eval complete")
    # DEMOING 
    # TODO
    # get map.pgm and map.yaml running cartographer my_robot.launch, choose a suitable start location for jackal, although doesnt matter to much
    # replace config/map/map.pgm and config/map/map.yaml
    # also get .pbstream file to run pure localiazation in cartographer in my_robot_localization.launch
    # NOTE if there's any issues in creating the map like jumping edit my_robot.lua in configurations
    # the agent subscribes to the SLAM pose estimiation topic, to make it subscribe to ODOMETRY instead do agent.navigation.POSE_SOURCE = 'ODOMETRY' instead of 'SLAM'

    # TODO
    # tune controller in config/controller_setting.json running agent.navigation.rotate(theta) # where theta is in radians, this should already be tuned though
        # agent.navigation.rotate(3.14/12)

    # TODO
    # validate if pure pursuit implementation is correct
    # perform simple navigation based on the map.yaml
    # run agent.navigation.navigate_to_goal(x, y) # choose a suitable x,y coordinate in the map.pgm to test this, the agent should rotate towards the first point then start path following
    # may have to tune main parmaeters
    # lookahead_distance in meters
    # constant forward veloicty m/s
    # MIN_LOOKAHEAD_ANGLE just ensure no 0 division dont worry about this
    # MAX_LOOKAHEAD_ANGLE tells the controller if the angle is to large only perform a pure rotation instead
    rospy.spin()
