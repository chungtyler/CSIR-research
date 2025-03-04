import warnings
from pathlib import Path

import cv2
import rospy
import numpy as np

from agent.perception import Perception
warnings.filterwarnings("ignore", category=FutureWarning)

if __name__ == "__main__":
    rospy.init_node('agent_node', anonymous=True)
    path_to_config = Path(__file__).resolve().parents[1] / 'config'
    perception = Perception(path_to_config)

    for i in range(10000):
        image = perception.rgb_image
        res = perception.get_object_data(image)
        # cv2.imwrite("RGB.png", image)
        print("Func res", res)
        # rospy.sleep(1)

    rospy.spin()