import os
os.environ['OLLAMA_HOST'] = "130.63.251.161:11434"
from ollama import chat

class DeepSeek:
    def __init__(self):
        self.model_name = 'deepseek-r1:7b'

    def get_response(self, prompt, query):
        response = chat(
            model=self.model_name, 
            messages=[
                {"role": "system", "content": prompt},
                {'role': 'user', 'content': query}
            ]
        )

        response = response['message']['content']
        return response


if __name__ == "__main__":
    system_prompt = "Assume you are a doctor skilled in treating patients."
    query = "I feel stomachache, can you help me?"

    ollma_client = DeepSeek()
    res = ollma_client.get_response(system_prompt, query)
    print(res)


