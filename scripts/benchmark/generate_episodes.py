import json
from pathlib import Path

if __name__ == "__main__":
    path_to_config = Path(__file__).resolve().parents[2] / 'benchmark/setup'

    # Load scenarios file
    with open(path_to_config / 'scenarios.json', 'r') as scenarios_file:
        scenarios = json.load(scenarios_file)

    
    # For each scenario generate episodes based on ground truth
    for id, data in scenarios:
        # get the ground truth and create ordered list of locations for each actor first actor is important to ground truth, other actors can be anywhere really, random
        # append to scenarios file with id of the scenario id,
        position = 0
        actor1 = 0
        scenario_id = 0
        format = {
            "scenario_id": {
                "main_actor": actor1,
                "settings": {
                    "actor1": position,
                    "actor2": position,
                    "actor3": position
                }
            }
        }
        
        #start_location['edges'].append({"target": goal_location['name'], "path_length": path_length})

    # Update locations file with path length information
    with open(path_to_config / 'episodes.json', 'w') as episodes_file:
        json.dump(format, episodes_file, indent=4)