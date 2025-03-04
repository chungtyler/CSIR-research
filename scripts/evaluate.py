# TODO run all the scenarios to benchmark performance of the framework

# Run through each scenario
# Run each episode (different settings)
# Run the agent with the prompt
# get task planner time to process time and if output is python executable record each
# get the average time and execution score for the scenario
# get the average time nad execution score for the benchmark

# perform for each module
# record in benchmark.json
import json
from pathlib import Path
from framework import Inference, TaskPlanner, LocationPredictor, PersonMatcher
from agent import Planning
import time
import rbo
from PIL import Image
import numpy as np
import os

import math
import random

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

class Agent:
    def __init__(self):
        pass

    def give_item(self):
        pass

    def take_item(self):
        pass

    def find_person(self, name):
        pass

def generate_path(start_position, goal_position):
    start = grid.node(start_position[0], start_position[1])
    goal = grid.node(goal_position[0], goal_position[1])
    path, _ = astar.find_path(start, goal, grid)
    return path

def get_actual_path(locations, true_goal):
    all_locations = locations.copy()
    all_locations.insert(0, 'start')
    actual_path_length = 0
    for i in range(len(all_locations)-1):
        start = all_locations[i]
        next = all_locations[i+1]

        start_position = locations_data[start]
        next_position = locations_data[next]
        actual_path = generate_path(start_position, next_position)
        actual_path_length += planning.get_path_length(actual_path)

        if next == true_goal:
            break

    return actual_path_length

def SPL(shortest_length_to_goal, actual_length_to_goal):
    SPL_score = shortest_length_to_goal / max(shortest_length_to_goal, actual_length_to_goal)
    return SPL_score * 100

def code_is_executable(code):
    code_error = ''
    try:
        for line in code:
            exec(line)
        code_status = "Passed"
    except Exception as error:
        code_status = "Failed"
        code_error = error
    return code_status, code_error

def evaluate_task_planner(query):
    previous_time = time.time()
    tasks, log_info = task_planner.get_tasks(query)
    processing_time = time.time() - previous_time

    code_status, error = code_is_executable(log_info['tasks'])
    log_info['code_stauts'] = code_status
    log_info['code_error'] = str(error)
    log_info['processing_time'] = processing_time
    return log_info

def evaluate_location_predictor(query, actor):
    previous_time = time.time()
    locations, log_info = location_predictor.get_locations(query, actor, [path_to_map])
    processing_time = time.time() - previous_time

    #log_info['response'] = 'o7'
    #code_status, error = code_is_executable(log_info['locations'])
    #log_info['code_stauts'] = code_status
    #log_info['code_error'] = str(error)
    log_info['processing_time'] = processing_time

    return log_info

def evaluate_person_matcher(actor, viewed_image):
    previous_time = time.time()
    does_person_match, log_info = person_matcher.match(actor, viewed_image)
    processing_time = time.time() - previous_time
    log_info['processing_time'] = processing_time
    return log_info


path_to_config = Path(__file__).resolve().parents[1] / 'config'
path_to_benchmark = Path(__file__).resolve().parents[1] / 'benchmark'

# Load episodes file
with open(path_to_config / 'map/locations.json', 'r') as locations_file:
    locations_data = json.load(locations_file)
    locations_data = locations_data['locations']

inference = Inference(path_to_config)
task_planner = TaskPlanner(path_to_config, inference)
location_predictor = LocationPredictor(path_to_config, inference)
person_matcher = PersonMatcher(path_to_config, inference)

planning = Planning(path_to_config)

astar = AStarFinder()
path_to_map = path_to_config / 'map/engineering_map.png'
map = Image.open(path_to_map)
map = map.convert("L")
map = np.array(map)
grid = Grid(matrix=map)

agent = Agent()

def random_locations(locations=['office', 'lab', 'storage_room', 'meeting_room', 'lunch_room']):
    random.shuffle(locations)

    log_info = {"locations": locations}
    return log_info

def get_closest_location(unexplored_locations, current_location):
    closest_disatnce = math.inf
    for location in unexplored_locations:
        path = generate_path(locations_data[current_location], locations_data[location])
        distance = planning.get_path_length(path)

        if distance < closest_disatnce:
            closest_disatnce = distance
            closest_location = location
    return closest_location

def greedy_location(locations=['office', 'lab', 'storage_room', 'meeting_room', 'lunch_room']):
    unexplored_locations = locations.copy()
    current_location = random.choice(unexplored_locations)
    ranked_locations = []
    unexplored_locations.remove(current_location)
    ranked_locations.append(current_location)
    while unexplored_locations:
        closest_locations = get_closest_location(unexplored_locations, current_location)
        unexplored_locations.remove(closest_locations)
        current_location = closest_locations
        ranked_locations.append(closest_locations)

    log_info = {"locations": ranked_locations}

    return log_info

#methods = ['gpt-4o', 'gpt-4o-mini', 'o1-mini', 'deepseek-r1:32b']
methods = ['gpt-4o-mini']#['greedy','random', 'o1-mini', 'deepseek-r1:32b']

if __name__ == '__main__':
    # Load scenarios file
    with open(path_to_benchmark / 'setup/scenarios.json', 'r') as scenarios_file:
        scenarios = json.load(scenarios_file)

    # Load episodes file
    with open(path_to_benchmark / 'setup/episodes.json', 'r') as episodes_file:
        episodes = json.load(episodes_file)

    task_planner_log_file = {}
    location_predictor_log_file = {}
    person_matcher_log_file = {}
    for method in methods:

        task_planner.model = method
        task_planner_log_file[method] = {}

        location_predictor_log_file[method] = {}

        person_matcher_log_file[method] = {}
        for scenario_ID, scenario_info in scenarios.items():
            query = scenario_info['scenario']
            task_planner_log_file[method][scenario_ID] = {}
            location_predictor_log_file[method][scenario_ID] = {}
            person_matcher_log_file[method][scenario_ID] = {}

            episodes_for_scenario = episodes[scenario_ID]
            actor = scenario_info['target_person']
            for episode_ID, actor_locations in episodes_for_scenario.items():
                print(f"Model: {method} || Scenario: {scenario_ID} || Episode: {episode_ID}")
                # task_planner_log = evaluate_task_planner(query)
                # task_planner_log_file[method][scenario_ID][episode_ID] = task_planner_log

                # if method == 'greedy':
                #     location_predictor_log = greedy_location()
                # elif method == 'random':
                #     location_predictor_log = random_locations()
                # else:
                #     location_predictor_log = evaluate_location_predictor(query, actor)

                # # Determine ranking accuracy
                # ground_truth = scenario_info['ground_truth']
                # location_predictor_log['ranking_accuracy'] = rbo.RankingSimilarity(ground_truth, location_predictor_log['locations']).rbo()

                # # Determine SPL
                # true_goal = actor_locations[actor]
                # shortest_path = generate_path(locations_data['start'], locations_data[true_goal])
                # shortest_length_to_goal = planning.get_path_length(shortest_path)

                # actual_length_to_goal = get_actual_path(location_predictor_log['locations'], true_goal)

                # location_predictor_log['SPL'] = SPL(shortest_length_to_goal, actual_length_to_goal)
                    
                # location_predictor_log_file[method][scenario_ID][episode_ID] = location_predictor_log
                # #if episode_ID == 'episode_1':
                #     #break

                for location in location_predictor_log['locations']:
                i = 0
                person_matcher_log_file[scenario_ID][episode_ID] = {}
                for location in ['lab', 'office', 'storage_room', 'lunch_room', 'meeting_room']:
                    viewed_actor_at_location = [key for key, value in actor_locations.items() if value == location]
                    if not viewed_actor_at_location:
                        continue
                    viewed_actor_at_location = viewed_actor_at_location[0]
                    viewed_image =  path_to_config / 'face_ID' / viewed_actor_at_location / 'image1.jpg'

                    person_matcher_log = evaluate_person_matcher(actor, viewed_image)

                    does_actor_actually_match = actor == viewed_actor_at_location
                    person_matcher_log['is_match_valid'] = does_actor_actually_match == person_matcher_log['does_person_match']
                    person_matcher_log['viewed_actor_at_location'] = viewed_actor_at_location

                    encounter_ID = f"encounter_{i}"
                    person_matcher_log_file[scenario_ID][episode_ID][encounter_ID] = person_matcher_log
                    i += 1

                    if does_actor_actually_match:
                        break
            #break
            time.sleep(1) # Prevent requests limit

        # with open(path_to_benchmark / 'evaluation/task_planner_data.json', 'w') as task_planner_data_file:
        #     json.dump(task_planner_log_file, task_planner_data_file, indent=4)

        with open(path_to_benchmark / 'evaluation/location_predictor_data.json', 'w') as location_predictor_data_file:
            json.dump(location_predictor_log_file, location_predictor_data_file, indent=4)

        # with open(path_to_benchmark / 'evaluation/person_matcher_data.json', 'w') as person_matcher_data_file:
        #     json.dump(person_matcher_log_file, person_matcher_data_file, indent=4)



        
    

    
