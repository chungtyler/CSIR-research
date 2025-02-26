import json

class PersonMatcher:
    # TODO implement VLM to perform the following:
    # Identify the social similarity to the performed action of the person in image
    # Identify how similar the face_ID is to the person
    # Validate based on these metrics if the person is indeed the described person
    # Return a true or false, confidence metric [0, 1],  and simple justification output
    # get a way to log the conversation, time it took to process, the finalized output,
    '''
    Person Matcher module used to determine if the target-person is the person in view
    based off of social role relevance and person visual descriptors
    '''
    def __init__(self, path_to_config, inference):
        # Reference to inference object
        self.inference = inference

        # Load Task Planner configuration
        with open(path_to_config + '/framework/person_matcher.json', 'r') as config_file:
            self.config = json.load(config_file)

        # Load model and prompt
        self.model = self.config['model']
        self.prompt = self.config['prompt']

    def get_tasks(self, query):
        # Use model inference to generate plan of executable code
        tasks = self.inference.get_response(self.model, self.prompt, query)
        return tasks
