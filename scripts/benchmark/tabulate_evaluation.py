import json
from pathlib import Path

if __name__ == '__main__':
    path_to_benchmark = Path(__file__).resolve().parents[2] / 'benchmark'

    # Load scenarios file
    with open(path_to_benchmark / 'setup/scenarios.json', 'r') as scenarios_file:
        scenarios = json.load(scenarios_file)

    # Load episodes file
    with open(path_to_benchmark / 'setup/episodes.json', 'r') as episodes_file:
        episodes = json.load(episodes_file)
    