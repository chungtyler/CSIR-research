from agent.agent import Agent

if __name__ == "__main__":
    agent = Agent()
    # DEMOING
    # TODO
    # get map.pgm and map.yaml running cartographer my_robot.launch, choose a suitable start location for jackal, although doesnt matter to much
    # replace config/map/map.pgm and config/map/map.yaml
    # also get .pbstream file to run pure localiazation in cartographer in my_robot_localization.launch
    # NOTE if there's any issues in creating the map like jumping edit my_robot.lua in configurations
    # the agent subscribes to the SLAM pose estimiation topic, to make it subscribe to ODOMETRY instead do agent.navigation.POSE_SOURCE = 'ODOMETRY' instead of 'SLAM'

    # TODO
    # tune controller in config/controller_setting.json running agent.navigation.rotate(theta) # where theta is in radians, this should already be tuned though

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
