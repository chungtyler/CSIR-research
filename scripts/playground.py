import pickle

import rospy
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


def eval_act(agent, path, controller):
    agent_pose = agent.navigation.pose
    print(agent_pose)
    linear, angular = controller.get_velocity_command(agent_pose, path)
    agent.navigation.set_velocity(linear, angular)

if __name__ == "__main__":
    record = False

    agent = Agent('SLAM')
    controller = agent.navigation.path_follow_control
    rospy.sleep(1)
    agent_pose = agent.navigation.pose
    last_pose = (-1000, -1000)

    if record:
        print("Record mode")
        path = []

        for i in range(1000):
            last_pose = record_act(agent, last_pose, path)
            rospy.sleep(0.01)
        with open("data.pkl", "wb") as f:
            pickle.dump(path, f)
            print(path)
            print("Record complete!")
    else:
        print("Reading from data")
        with open("data.pkl", "rb") as f:
            path = pickle.load(f) 

        path = path[::-1]
        for i in range(100):
            eval_act(agent, path, controller)
            rospy.sleep(0.1)
        print(path)
        print("Eval complete")
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
