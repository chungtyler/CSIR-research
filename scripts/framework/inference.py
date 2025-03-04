import json
from models import OpenAI, DeepSeek

class Inference:
    def __init__(self, path_to_config):
        # Load API Keys
        with open(path_to_config / 'api_keys.json', 'r') as api_key_file:
            self.api_keys = json.load(api_key_file)

        # Load Models
        self.models = {
            'OpenAI': OpenAI(self.api_keys['OpenAI']),
            'DeepSeek': DeepSeek()
        }

    def get_response(self, model, prompt, query, images=None):
        # Get the response of the defined model
        if 'gpt' in model or 'o1' in model: # Run OpenAI model
            response = self.models['OpenAI'].get_response(model, prompt, query, images)

        elif 'deepseek' in model: # Run DeepSeek model
            response = self.models['DeepSeek'].get_response(model, prompt, query)

        return response
