
import json
import ast
import re
import random

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
            self.locations_data = json.load(locations_file)
            self.locations = [location for location in self.locations_data['locations']]

        self.location_positions = self.locations_data['locations']
        self.distance_costs = self.locations_data['costs']

        self.formatted_location_positions = ''
        for name, coordinate in self.location_positions.items():
            self.formatted_location_positions += f"{name}: {coordinate}\n"

        self.formatted_distance_costs = ''
        for name, cost in self.distance_costs.items():
            self.formatted_distance_costs += f"{name}: {cost}\n"

        # Load actor information
        with open(path_to_config / 'actors.json', 'r') as actors_file:
            self.actors = json.load(actors_file)

        # Load Task Planner configuration
        with open(path_to_config / 'framework/location_predictor.txt', 'r') as prompt_file:
            self.prompt_template = prompt_file.read()

        # Load Task Planner configuration
        with open(path_to_config / 'framework/framework_models.json', 'r') as framework_models_file:
            framework_models = json.load(framework_models_file)
            self.model = framework_models['location_predictor']

    def response_to_locations(self, response):
        locations = re.findall(r'<list>(.*?)</list>', response, re.DOTALL)
        locations = ast.literal_eval(locations[-1].replace(' ',''))
        return locations
    
    def format_actor_prompt(self, actor):
        actor_info = self.actors[actor.lower()]
        occupation = actor_info['occupation']
        responsibility = actor_info['responsibility']

        formatted_actor_prompt = f"Name: {actor.capitalize()}\nOccupation: {occupation}\nResponsibility: {responsibility}"
        return formatted_actor_prompt

    def format_prompt(self, actor):
        locations_list = self.locations.copy()
        locations_list.remove('start')
        randomized_list = locations_list.copy()
        random.shuffle(randomized_list)
        formatted_actor_prompt = self.format_actor_prompt(actor)
        hallway = 'The northern hallway connects to the meeting_room, storage_room, and office, while the southern hallway connects to the lab and lunch_room'
        formatted_prompt = self.prompt_template.format(locations=locations_list,hallway=hallway, location_positions=self.formatted_location_positions, distance_costs=self.formatted_distance_costs, actor_info=formatted_actor_prompt, randomized=randomized_list)
        return formatted_prompt

    def get_locations(self, query, actor, map=None):
        # Use model inference to generate plan of executable code
        formatted_prompt = self.format_prompt(actor)
        response = self.inference.get_response(self.model, formatted_prompt, query, map)

        try:
            locations = self.response_to_locations(response)
            is_xml_format = True
            xml_error = ''
        except Exception as error:
            locations = ''
            is_xml_format = False
            xml_error = error

        log_info = {
            "response": response,
            "locations": locations,
            "is_xml_format": is_xml_format,
            "xml_error": str(xml_error)
        }
        return locations, log_info
