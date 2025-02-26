import json
from pathlib import Path
from agent import Planning

if __name__ == "__main__":
    path_to_config = Path(__file__).resolve().parents[2] / 'config'
    planning = Planning(path_to_config)

    # Load locations file
    with open(path_to_config / 'map/locations.json', 'r') as locations_file:
        locations_data = json.load(locations_file)
        locations = locations_data['locations']

    # For each node determine the distance to each location
    for start_location in locations:
        start_position = start_location['position']
        for goal_location in locations:
            if start_location['name'] is goal_location['name']: # Continue if the start and goal node are the same
                continue

            goal_position = goal_location['position'] # Get the goal locations position
            path = planning.generate_path(start_position, goal_position) # Generate pixel path
            real_path = planning.convert_to_real_path(path) # Convert to real path in meters
            path_length = planning.get_path_length(real_path) # Calculate the distance between the
            # Save positions

    # Write to file

