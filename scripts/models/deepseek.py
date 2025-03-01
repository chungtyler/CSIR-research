import os
os.environ['OLLAMA_HOST'] = "130.63.251.161:11434"
from ollama import chat

class DeepSeek:
    def __init__(self):
        self.model_name = 'deepseek-r1:32b'

    def get_response(self, model, prompt, query):
        response = chat(
            model=model, 
            messages=[
                {"role": "system", "content": prompt},
                {'role': 'user', 'content': query}
            ]
        )

        response = response['message']['content']
        return response


if __name__ == "__main__":
    #system_prompt = "Assume you are a doctor skilled in treating patients. Keep your response within a paragraph."
    #query = "I feel stomachache, can you help me?"
    system_prompt = "You are an AI Assistant here to answer the questions of the user."
    query = "What is the capital of France?"

    ollma_client = DeepSeek()
    res = ollma_client.get_response(system_prompt, query)
    print(res)


