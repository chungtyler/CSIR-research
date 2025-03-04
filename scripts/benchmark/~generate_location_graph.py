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
    path_to_config = Path(__file__).resolve().parents[2] / 'config'
    path_to_map = path_to_config / 'map/engineering_map.png'

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

    # For each node determine the distance to each location
    for start_location in locations:
        start_position = start_location['position']
        start_location['edges'] = []
        for goal_location in locations:
            if start_location['name'] is goal_location['name']: # Continue if the start and goal node are the same
                continue

            # Get path length between locations
            goal_position = goal_location['position'] # Get the goal locations position
            path = generate_path(astar, grid, start_position, goal_position) # Generate pixel path
            #real_path = planning.convert_to_real_path(path) # Convert to real path in meters
            path_length = planning.get_path_length(path) # Calculate the distance between the
            planning.show_path(map, path)
            
            # Update locations edge
            start_location['edges'].append({"target": goal_location['name'], "path_length": path_length})

    # Update locations file with path length information
    with open(path_to_config / 'map/locations.json', 'w') as locations_file:
        json.dump(locations_data, locations_file, indent=4)

