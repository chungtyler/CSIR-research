from pathlib import Path

import cv2
import rospy
import numpy as np

from agent import Planning
from agent.navigation import Navigation

if __name__ == "__main__":
    rospy.init_node('agent_node', anonymous=True)
    path_to_config = Path(__file__).resolve().parents[1] / 'config'
    navigation = Navigation(path_to_config, "SLAM")
    planning = Planning(path_to_config, 0.20) # 0.25

    # planning.map_origin[2] = 3.14/8
    agent_position = [navigation.pose['X'], navigation.pose['Y']] # [0, 3]
    print(agent_position)
    goal_position = [1, 1]
    map = planning.map    

    agent_pixel_positon = planning.convert_to_pixel_point(agent_position)
    goal_pixel_position = planning.convert_to_pixel_point(goal_position)
    print(agent_pixel_positon, goal_pixel_position)

    display_map = map.copy()
    display_map = display_map.astype(np.uint8)
    display_map = cv2.cvtColor(display_map, cv2.COLOR_GRAY2RGB)
    display_map[agent_pixel_positon[0]][agent_pixel_positon[1]] = (255, 0, 0)
    display_map[goal_pixel_position[0]][goal_pixel_position[1]] = (255, 0, 255)
    
    pixel_path = planning.generate_path(agent_pixel_positon, goal_pixel_position)
    real_path = planning.convert_to_real_path(pixel_path)
    # planning.show_path(map, pixel_path)
    # planning.show_map(map)
    if len(real_path) > 0:
        path_length = planning.get_path_length(real_path)
        planning.show_path(map, pixel_path)
        print(path_length)
    else:
        print("Cannot reach the destination!")

    rospy.spin()