import json
import numpy as np
from PIL import Image
from pathlib import Path
from agent.planning import Planning
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import time

def generate_path(astar, grid, start_position, goal_position):
    start = grid.node(start_position[0], start_position[1])
    goal = grid.node(goal_position[0], goal_position[1])
    path, _ = astar.find_path(start, goal, grid)
    return path

if __name__ == "__main__":
    path_to_config = Path(__file__).resolve().parents[1] / 'config'
    path_to_map = path_to_config / 'map/map.pgm'

    map = Image.open(path_to_map)
    map = map.convert("L")
    map = np.array(map)
    print(map.shape)

    planning = Planning(path_to_config)
    astar = AStarFinder()
    grid = Grid(matrix=map)

    # Load locations file
    with open(path_to_config / 'map/locations.json', 'r') as locations_file:
        locations_data = json.load(locations_file)
        locations = locations_data['locations']

    #print(locations[])
    locations_data['costs'] = {}
    costs = locations_data['costs']
    # For each node determine the distance to each location
    for start_name, start_position in locations.items():
        for goal_name, goal_position in locations.items():
            print(goal_name)
            if goal_name is start_name: # Continue if the start and goal node are the same
                continue

            print(locations_data['costs'])

            duplicate_cost = False
            for cost in locations_data['costs']:
                print(f"Start: {start_name} || Goal: {goal_name} || Cost: {cost}")
                if start_name in cost and goal_name in cost:
                    duplicate_cost = True

            if duplicate_cost:
                continue

            # Get path length between locations
            path = generate_path(astar, grid, start_position, goal_position) # Generate pixel path
            #real_path = planning.convert_to_real_path(path) # Convert to real path in meters
            path_length = planning.get_path_length(path) # Calculate the distance between the
            planning.show_path(map, path)
            
            # Update locations edge
            key = f"{goal_name} and {start_name}"
            value = int(path_length)
            locations_data['costs'][key] = value

    locations_data['costs'] = {k: v for k, v in sorted(locations_data['costs'].items(), key=lambda item: item[1], reverse=True)}
    # Update locations file with path length information
    with open(path_to_config / 'map/locations.json', 'w') as locations_file:
        json.dump(locations_data, locations_file, indent=4)

