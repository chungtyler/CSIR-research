import json

class TaskPlanner:
    '''
    Task Planner module used to generate the list of commands to achieve the goal
    Uses a function library to as wrappers to lower level control commands
    '''
    def __init__(self, path_to_config, inference):
        # Reference to inference object
        self.inference = inference

        # Load Task Planner configuration
        with open(path_to_config + '/framework/task_planner.json', 'r') as config_file:
            self.config = json.load(config_file)

        # Load model and prompt
        self.model = self.config['model']
        self.prompt = self.config['prompt']

    def get_tasks(self, query):
        # Use model inference to generate plan of executable code
        tasks = self.inference.get_response(self.model, self.prompt, query)
        return tasks
