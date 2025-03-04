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
        self.query = 'Does the person in the given image match the person in the face ID image?'
        self.path_to_face_ID = path_to_config / 'face_ID'

        # Load actor information
        with open(path_to_config / 'actors.json', 'r') as actors_file:
            self.actors = json.load(actors_file)

        # Load Task Planner configuration
        with open(path_to_config / 'framework/person_matcher.txt', 'r') as prompt_file:
            self.prompt = prompt_file.read()

        # Load Task Planner configuration
        with open(path_to_config / 'framework/framework_models.json', 'r') as framework_models_file:
            framework_models = json.load(framework_models_file)
            self.model = framework_models['person_matcher']

    def get_face_ID(self, actor):
        face_ID = self.path_to_face_ID / actor.lower() / 'face_ID.jpg'
        return face_ID
    
    def does_person_match(self, person_info):
        does_person_match = None
        if "true" in person_info.lower():
            does_person_match = True
        elif "false" in person_info.lower():
            does_person_match = False
        
        return does_person_match

    def match(self, actor, image):
        # Use model inference to generate plan of executable code
        face_ID = self.get_face_ID(actor)
        response = self.inference.get_response(self.model, self.prompt, self.query, [face_ID, image])

        try:
            does_person_match = self.does_person_match(response)
            if does_person_match is None:
                raise ValueError("No boolean in output")
            output_has_boolean = True
            output_error = ''
        except Exception as error:
            does_person_match = ''
            output_has_boolean = False
            output_error = error

        log_info = {
            "response": response,
            "does_person_match": does_person_match,
            "output_has_boolean": output_has_boolean,
            "output_error": str(output_error)
        }
        return does_person_match, log_info
