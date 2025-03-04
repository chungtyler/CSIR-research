import json
from pathlib import Path
import math
import random

def get_target_location(ground_truth):
    weights = [math.exp(-x) for x in range(1, len(ground_truth)+1)]
    target_location = random.choices(ground_truth, weights=weights)
    return target_location[0]

def get_remaining_actor_locations(locations, actors):
    sample_number = random.randint(0, len(actors))
    random_actors = random.sample(actors, sample_number)
    random_locations = random.sample(locations, sample_number)
    return random_actors, random_locations

if __name__ == "__main__":
    path_to_benchmark = Path(__file__).resolve().parents[2] / 'benchmark/setup'
    path_to_config = Path(__file__).resolve().parents[2] / 'config'

    # Load scenarios file
    with open(path_to_benchmark / 'scenarios.json', 'r') as scenarios_file:
        scenarios = json.load(scenarios_file)

    # Load actors file
    with open(path_to_config / 'actors.json', 'r') as actors_file:
        actors = json.load(actors_file)
        all_actors_names = [actor for actor in actors]

    # Load locations file
    with open(path_to_config / 'map/locations.json', 'r') as locations_file:
        locations = json.load(locations_file)
        location_names = [location for location in locations['locations'] if location != 'start']

    # Load episodes file
    with open(path_to_benchmark / 'episodes.json', 'r') as episodes_file:
        episodes = json.load(episodes_file)

    episodes = {}
    episod_number = 5
    for scenario_ID, scenario_info in scenarios.items():
        episodes[scenario_ID] = {}
        for i in range(episod_number):
            target_person = scenario_info['target_person']
            actor_names = all_actors_names.copy()
            actor_names.remove(target_person)

            available_locations = location_names.copy()

            ground_truth = scenario_info['ground_truth']
            target_location = get_target_location(available_locations)
            available_locations.remove(target_location)

            random_actors, random_locations = get_remaining_actor_locations(available_locations, actor_names)

            episode = {target_person: target_location}
            if random_actors or random_locations:
                for j in range(len(actor_names)-1):
                    print(random_actors)
                    episode[random_actors[j]] = random_locations[j]

            episode_ID = f"episode_{i}"
            episodes[scenario_ID][episode_ID] = episode

    # Update locations file with path length information
    with open(path_to_benchmark / 'episodes.json', 'w') as episodes_file:
        json.dump(episodes, episodes_file, indent=4)