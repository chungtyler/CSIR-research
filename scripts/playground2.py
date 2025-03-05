from framework import Inference
from pathlib import Path
from agent.agent import Agent

path_to_config = Path(__file__).resolve().parents[1] / 'config'
inference = Inference(path_to_config)

# model = 'deepseek-r1:32b'
# prompt = 'You are an AI chat assistant here to answer the user question'
# query = 'How many apples do I have if i start with 10, give away 5, and recieve 2?'

# response = inference.get_response(model, prompt, query)
# print("USER:", query)
# print("SYSTEM:", response)
import rospy
import time
agent = Agent(path_to_config)
loc = agent.navigation.planning.convert_to_pixel_point([0,0]) #147, 49
print(loc)
# i = 0
# while not rospy.is_shutdown():
#     print(i)
#     agent.look_at_person()
#     #rospy.sleep(0.5)
#     i +=1


