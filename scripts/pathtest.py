from pathlib import Path
from agent import Planning

if __name__ == "__main__":
    path_to_config = Path(__file__).resolve().parents[1] / 'config'
    planning = Planning(path_to_config, 0.0)

    #planning.map_origin[2] = 3.14/8

    agent_position = [0, 0]
    goal_position = [-2, 3]
    agent_pixel_positon = planning.convert_to_pixel_point(agent_position)
    goal_pixel_position = planning.convert_to_pixel_point(goal_position)

    map = planning.map
    
    pixel_path = planning.generate_path(agent_pixel_positon, goal_pixel_position)
    real_path = planning.convert_to_real_path(pixel_path)

    planning.show_path(map, pixel_path)
    #planning.show_map(map)
    path_length = planning.get_path_length(real_path)
    print(path_length)
