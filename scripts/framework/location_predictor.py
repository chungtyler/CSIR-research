import json
import ast
import re

class LocationPredictor:
    '''
    Location Predictor module used to determine an optimal order of locations to travel
    to based on person likley hood of being there using social and contextual Information
    as well as the distance cost between locations
    '''
    def __init__(self, path_to_config, inference):
        # Reference to inference object
        self.inference = inference

        # Load Task Planner configuration
        with open(path_to_config / 'framework/location_predictor.json', 'r') as config_file:
            self.config = json.load(config_file)

        # Load model and prompt
        self.model = self.config['model']
        self.prompt = self.config['prompt']

        # Load location information
        with open(path_to_config / 'map/locations.json', 'r') as locations_file:
            self.locations = json.load(locations_file)

        # Load actor information
        with open(path_to_config / 'actors.json', 'r') as actors_file:
            self.actors = json.load(actors_file)

    def response_to_locations(self, response):
        locations = re.findall(r'<list>(.*?)</list>', response, re.DOTALL)
        locations = ast.literal_eval(locations[-1].replace(' ',''))
        return locations

    def get_locations(self, query):
        # Use model inference to generate plan of executable code
        response = self.inference.get_response(self.model, self.prompt, query)
        locations = self.response_to_locations(response)
        return locations
