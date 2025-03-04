from agent.agent import Agent

if __name__ == "__main__":
    agent = Agent('SLAM')

    query = "Can you give this soda can to Tyson"
    tasks, _ = agent.task_planner.get_tasks(query)

    try:
        for task in tasks:
            exec(task)
        print("Tasks Executed")
    except Exception as error:
        print("Failed exception", error)

