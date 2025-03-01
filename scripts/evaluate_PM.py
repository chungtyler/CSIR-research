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

def evaluate_person_matcher(actor, viewed_image):
    previous_time = time.time()
    does_person_match, log_info = person_matcher.match(actor, viewed_image)
    processing_time = time.time() - previous_time
    log_info['processing_time'] = processing_time
    return log_info


path_to_config = Path(__file__).resolve().parents[1] / 'config'
path_to_benchmark = Path(__file__).resolve().parents[1] / 'benchmark'

# Load person matcher test file
with open(path_to_config / 'person_matcher_test/locations.json', 'r') as locations_file:
    locations_data = json.load(locations_file)
    locations_data = locations_data['locations']

inference = Inference(path_to_config)
person_matcher = PersonMatcher(path_to_config, inference)

methods = ['gpt-4o-mini']#['greedy','random', 'o1-mini', 'deepseek-r1:32b']
actors = ['tyson', 'grace', 'jerry']

if __name__ == '__main__':
    # Load scenarios file
    with open(path_to_benchmark / 'setup/scenarios.json', 'r') as scenarios_file:
        scenarios = json.load(scenarios_file)

    # Load episodes file
    with open(path_to_benchmark / 'setup/episodes.json', 'r') as episodes_file:
        episodes = json.load(episodes_file)

    person_matcher_log_file = {}
    for method in methods:

        person_matcher_log_file[method] = {}

        for actor in actors
       
            person_matcher_log_file[method][scenario_ID] = {}

            episodes_for_scenario = episodes[scenario_ID]
            actor = scenario_info['target_person']
            for episode_ID, actor_locations in episodes_for_scenario.items():
                print(f"Model: {method} || Scenario: {scenario_ID} || Episode: {episode_ID}")

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


        with open(path_to_benchmark / 'evaluation/person_matcher_data.json', 'w') as person_matcher_data_file:
            json.dump(person_matcher_log_file, person_matcher_data_file, indent=4)



        
    

    
