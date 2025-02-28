from pathlib import Path
from agent import Planning

if __name__ == "__main__":
    path_to_config = Path(__file__).resolve().parents[1] / 'config'
    planning = Planning(path_to_config, 0.20) # 0.25

    #planning.map_origin[2] = 3.14/8

    agent_position = [0, 3]
    goal_position = [4, 2.5]
    agent_pixel_positon = planning.convert_to_pixel_point(agent_position)
    print(agent_pixel_positon)
    goal_pixel_position = planning.convert_to_pixel_point(goal_position)
    print(goal_pixel_position)

    map = planning.map
    print(map[49][80], map[49][100])
    
    pixel_path = planning.generate_path(agent_pixel_positon, goal_pixel_position)
    real_path = planning.convert_to_real_path(pixel_path)
    planning.show_path(map, pixel_path)
    # planning.show_map(map)
    if len(real_path) > 0:
        path_length = planning.get_path_length(real_path)
        print(path_length)
