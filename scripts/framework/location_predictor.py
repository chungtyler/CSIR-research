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

        # Load location information
        with open(path_to_config / 'map/locations.json', 'r') as locations_file:
            self.locations = json.load(locations_file)

        # Load actor information
        with open(path_to_config / 'actors.json', 'r') as actors_file:
            self.actors = json.load(actors_file)

        # Load Task Planner configuration
        with open(path_to_config / 'framework/location_predictor.txt', 'r') as prompt_file:
            unformatted_prompt = prompt_file.read()

        print(unformatted_prompt)
        self.prompt = unformatted_prompt.format(locations=self.locations, 
                                                formatted_actor_info=self.actors,
                                                randomized=self.locations,
                                                )
        
        print(self.prompt)

        # Load Task Planner configuration
        with open(path_to_config / 'framework/framework_models.json', 'r') as framework_models_file:
            framework_models = json.load(framework_models_file)
            self.model = framework_models['location_predictor']

    def response_to_locations(self, response):
        locations = re.findall(r'<list>(.*?)</list>', response, re.DOTALL)
        locations = ast.literal_eval(locations[-1].replace(' ',''))
        return locations

    def get_locations(self, query):
        # Use model inference to generate plan of executable code
        response = self.inference.get_response(self.model, self.prompt, query)
        print(response)
        locations = self.response_to_locations(response)
        return locations
