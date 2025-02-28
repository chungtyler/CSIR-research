from framework import TaskPlanner, PersonMatcher, LocationPredictor, Inference
from pathlib import Path

if __name__ == '__main__':
    path_to_config = Path(__file__).resolve().parents[1] / 'config'
    inference = Inference(path_to_config)
    #task_planner = TaskPlanner(path_to_config, inference)
    location_predictor = LocationPredictor(path_to_config, inference)
    prompt = ''''
    You are the terminator speak like a robot!
    '''
    query = 'Can you give this apple to Tyson'

    #response = inference.get_response('gpt-4o', prompt, query)
    #tasks = task_planner.get_tasks(query)
    locations = location_predictor.get_locations('Can you give this apple to Tyson?')
    print(prompt)
    #print(query)
    #print(tasks)
    print(locations)
