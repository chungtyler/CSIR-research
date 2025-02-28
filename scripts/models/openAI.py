import validators
import os
import base64
from openai import OpenAI as openAI

class OpenAI:
    '''
    OpenAI class to get responses from the available models (text and image)
    '''
    def __init__(self, API_KEY):
        self.API_KEY = API_KEY
        print(type(API_KEY))
        self.client = openAI(api_key=API_KEY)

    def format_image(self, image):
        # Process the image to format it to the correct type for OpenAI
        if validators.url(image): # If already image url send back
            image_url = image
        elif os.path.exists(image): # If local directory conver to base 64 and upload
            with open(image) as image_file:
                base64_image = base64.b64encode(image_file.read()).decord('UTF-8')

            image_url = f"data:image/jpg;base64,{base64_image}"
        else: # Otherwise convert image to base64
            image_url = f"data:image/jpg;base64,{image}"

        return image_url

    def get_response(self, model, prompt, query, images=None):
        # Get the user response with defined system prompt behaviour
        user_content = [{"type": "text","text": query}]

        # Add image content for multiple images
        if images:
            for image in images:
                formatted_image = self.format_image(image)
                user_content.append({"type": "image_url", "image_url": {"url": formatted_image}})

        messages = [{"role": "system", "content": prompt}, {"role": "user", "content": user_content}]

        # Send message payload and return the messages response
        completion = self.client.chat.completions.create(model=model, messages=messages)
        response = completion.choices[0].message.content
        return response
