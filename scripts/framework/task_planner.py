import json
import re

class TaskPlanner:
    '''
    Task Planner module used to generate the list of commands to achieve the goal
    Uses a function library to as wrappers to lower level control commands
    '''
    def __init__(self, path_to_config, inference):
        # Reference to inference object
        self.inference = inference

        # Load Task Planner configuration
        with open(path_to_config / 'framework/task_planner.txt', 'r') as prompt_file:
            self.prompt = prompt_file.read()

        print(self.prompt)
        # Load Task Planner configuration
        with open(path_to_config / 'framework/framework_models.json', 'r') as framework_models_file:
            framework_models = json.load(framework_models_file)
            self.model = framework_models['task_planner']

    def get_tasks(self, query):
        # Use model inference to generate plan of executable code
        response = self.inference.get_response(self.model, self.prompt, query)
        tasks = re.findall(r'<command>(.*?)</command>', response, re.DOTALL)
        tasks = [task.replace(' ', '') for task in tasks]
        return tasks
