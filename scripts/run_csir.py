from agent.agent import Agent
import rospy
if __name__ == "__main__":
    agent = Agent('SLAM')
    query = "Can you give this soda can to Tyson"
    tasks, _ = agent.task_planner.get_tasks(query)

    # rospy.sleep(1)
    # agent.navigation.set_velocity(1,0)
    # rospy.sleep(0.5)
    # agent.navigation.set_velocity(-1,0)
    # rospy.sleep(0.5)

    print(tasks)
    # breakpoint()
    # try:
    for task in tasks:
        exec(task)
    print("Tasks Executed")
    # except Exception as error:
    #     print("Failed exception", error)

    rospy.spin()