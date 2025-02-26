import rospy
from agent.agent import Agent

if __name__ == "__main__":
    agent = Agent()
    controller = agent.navigation.path_follow_control
    rospy.sleep(1)
    agent_pose = agent.navigation.pose
    path = [(2.8433975675761776, 0.6262624244218765), (2.8923501214664653, 0.6313319041445965), (2.943383852372716, 0.6369919713929902), (2.9952301811902786, 0.6436534532546582), (3.044123001170995, 0.6504965045476414), (3.0945967059497854, 0.657858895340871), (3.145749049491814, 0.6655649674852228), (3.1941555609586842, 0.6731761657486489), (3.245807253873674, 0.6811984160852008), (3.297151287304246, 0.6889732662426342), (3.34938131810409, 0.6965666486622559), (3.3993167654468617, 0.7035994581680893), (3.451060167147765, 0.7107746880782142), (3.5031830271954325, 0.7180838137258604), (3.553193452246572, 0.725176558529808), (3.604633303506985, 0.7325772186307283), (3.6553925642302936, 0.7399956283587007), (3.707106493903372, 0.7476343608460525), (3.7548638875647704, 0.7544155274534974), (3.804466861058995, 0.761473015305944), (3.8530259084376235, 0.768156935118193), (3.8961028959957904, 0.7738543331475237), (3.946286785836842, 0.7802312076583429), (3.9965362614771003, 0.786081321720042), (4.044810423841188, 0.7912856637149075), (4.091847290217559, 0.7961292553434947), (4.141353629858878, 0.8008484630019826), (4.1913600262979935, 0.8054827782960328), (4.24014347680032, 0.8097686718035365), (4.290016874746268, 0.8136102167169117), (4.338651333876932, 0.8168589136767428), (4.38748484655798, 0.8195562268166267), (4.440387517767848, 0.8219771518356307), (4.491897247652123, 0.8238439230777925), (4.543235471561713, 0.8250691257541759), (4.596052775231978, 0.8259899929441059), (4.648131897075371, 0.8265164994318942), (4.7011554017662185, 0.8266412250304166), (4.751559058356518, 0.826637509767161), (4.802620464378175, 0.8268155706268615), (4.856745009677146, 0.8273316292439595), (4.908997023281491, 0.8280203387771612), (4.960186460334266, 0.8292317014051691), (5.012607802686723, 0.8308387539448877), (5.066556101143473, 0.8330543069765591), (5.120413521441914, 0.8356801945672085), (5.172948431721628, 0.8384996292059356), (5.223719978715529, 0.8413527795100711), (5.27066963405512, 0.843952309033383), (5.3187787548472745, 0.8469274095863656)]
    path.reverse()
    # path = path[:-1]
    print(path)
    for i in range(50):
        agent_pose = agent.navigation.pose
        print(agent_pose)
        linear, angular = controller.get_velocity_command(agent_pose, path)
        # agent.navigation.rotate(rot)
        # rot += 3.14 / 6
        # agent.navigation.set_velocity(0.2, 0)
        path.append((agent_pose['X'], agent_pose['Y']))
        agent.navigation.set_velocity(linear, angular)
        rospy.sleep(0.1)
    print(path)
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
    print("Action complete")
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
