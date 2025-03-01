from deepface import DeepFace
import json
from pathlib import Path

path_to_test = Path(__file__).resolve().parents[1] / 'config/person_matcher_test'

actor = 'tyson'
image = path_to_test / actor / 'face_ID.jpg'
result = DeepFace.analyze(image)
print(result)