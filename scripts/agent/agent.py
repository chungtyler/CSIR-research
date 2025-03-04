import rospy
import json
from pathlib import Path

from .navigation import Navigation
from .perception import Perception
from framework import PersonMatcher, LocationPredictor, TaskPlanner, Inference

class Agent:
    '''
    Agent class handling the perception and navigation stack as well as the framework for PersonNav
    '''
    def __init__(self, POSE_SOURCE='ODOMETRY'):
        rospy.init_node('agent_node', anonymous=True)

        # Get absolute path to config file
        path_to_config = Path(__file__).resolve().parents[2] / 'config'

        # Instantiate agent stack
        self.perception = Perception(path_to_config)
        self.navigation = Navigation(path_to_config, POSE_SOURCE=POSE_SOURCE) # Can change pose information based on 'SLAM' as well

        # Instantiate Person Goal Navigation framework stack
        #self.inference = Inference(path_to_config)
        #self.task_planner = TaskPlanner(path_to_config, self.inference)
        #self.location_predictor = LocationPredictor(path_to_config, self.inference)
        #self.person_matcher = PersonMatcher(path_to_config, self.inference)

        # Load locations file
        with open(path_to_config / 'map/locations.json', 'r') as locations_file:
            self.locations = json.load(locations_file)

        # Load actors file
        with open(path_to_config / 'actors.json', 'r') as actors_file:
            self.actors = json.load(actors_file)

        #rospy.spin()

    def track_person(self, iterations):
        for _ in range(iterations):
            self.look_at_person(0.5)
            rospy.sleep(0.2)


    def look_at_person(self, min_confidence, threhsold=75):
        image = self.perception.rgb_image
        _, width, _ = image.shape
        while not rospy.is_shutdown():
            object_info = self.perception.get_object_data(image)
            if not object_info:
                rospy.sleep(0.1)
                continue

            if object_info['confidence'] > min_confidence:
                x, y = self.perception.get_object_ROI(object_info)
                error = width/2 - x

                if abs(error) < threhsold:
                    return
                
                self.navigation.set_velocity(0, 0.001*error + 0.01)


    def give_item(self):
        # Agent visual movement for giving item action
        for _ in range(3):
            self.navigation.set_velocity(-0.2,0)
            rospy.sleep(0.35)
            self.navigation.set_velocitymove(0.2,0)
            rospy.sleep(0.35)

    def take_item(self):
        # Agent visual movement for taking item action
        for _ in range(3):
            self.navigation.set_velocity(-0.2,0)
            rospy.sleep(0.35)
            self.navigation.set_velocitymove(0.2,0)
            rospy.sleep(0.35)

    def find_person(self, target_person):
        # TODO
        # Get the context prompt
        # Feed context prompt, target_person_info, locations info -> location predictor -> output the scores of each location
        # Loop the locations based on score order
            # make agent travel to location
            # check if there's a person in FOV
                # if so move towards person by X
                # look at person
                # take image send to -> person_matcher -> boolean
                # if person_matches:
                    # set target person to TRUE

        # ensure to log the time to process for location and person matcher, the total distance the agent moved, ranked list, location of person, method type
        # return if the person is found true or false
        pass

if __name__ == "__main__":
    try:
        Agent()
    except rospy.ROSInterruptException:
        pass
