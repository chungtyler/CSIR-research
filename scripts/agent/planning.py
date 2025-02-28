#import rospy
import cv2
import numpy as np
import yaml
import math
from PIL import Image
#mport pyastar2d as astar
#from Astar import AStar
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
#from astar import AStar

class Planning:
    '''
    Path planning class handling waypoint creation and navigatio to each waypoint
    '''
    def __init__(self, path_to_config, agent_radius=0):
        # Initialize occupancy map and path planning
        self.agent_radius = agent_radius
        self.map, self.map_resolution, self.map_origin = self.load_map_config(path_to_config / 'map/map.pgm', path_to_config / 'map/map.yaml')
        self.astar = AStarFinder()
        self.PATH_COLOUR = (255, 0, 255)

    def load_map_config(self, map_pgm_path, map_yaml_path):
        # Read and record map resolution [meters / pixel] and origin
        with open(map_yaml_path, 'r') as map_yaml:
            map_data = yaml.safe_load(map_yaml)

        map_resolution = map_data['resolution']
        map_origin = map_data['origin']

        # Get map.pgm and map.yaml to create binary map with known resolution and origin
        map_pgm = Image.open(map_pgm_path)
        map = np.array(map_pgm)
        map = np.where(map <= 205, 0, 255.0)

        agent_pixel_radius = round(self.agent_radius / map_resolution)
        padded_map = self.pad_map(map, agent_pixel_radius)
        self.grid = Grid(matrix=padded_map)

        return padded_map, map_resolution, map_origin

    def pad_map(self, map, radius):
        # Pad the occupied areas of the map with defined radius to prevent agent collision
        padded_map = map.copy()
        for rows in range(len(padded_map)):
            for cols in range(len(padded_map[0])):
                if map[rows][cols] == 0:
                    cv2.circle(padded_map, [cols, rows], radius, 0.0, -1)

        return padded_map

    def show_map(self, map, scaling_factor=1):
        # Display the map and resize it accordingle for visualization
        map_shape = map.shape
        map_shape_resized = (round(map_shape[1]*scaling_factor), round(map_shape[0]*scaling_factor)) # Resize aspect ratio or shape of the map
        map_resized = cv2.resize(map, map_shape_resized, interpolation=cv2.INTER_CUBIC) # Apply scaling to the map

        # Display map
        cv2.imshow("Occupancy Map", map_resized)
        cv2.waitKey(5000)
        cv2.destroyAllWindows()

    def generate_path(self, start, goal):
        # Get AStar path as a list of tuples (x, y)
        #ath = astar.astar_path(self.map, start, goal, allow_diagonal=True)
        #path = AStar(self.map, start, goal)
        #path = self.astar.find_path(start, goal)
        start_node = self.grid.node(start[1], start[0])
        goal_node = self.grid.node(goal[1], goal[0])
        path, _ = self.astar.find_path(start_node, goal_node, self.grid)
        return path

    def get_path_length(self, path):
        # return path_length by finding the distance between points and summing the total length
        path_length = 0
        previous_point = path[0]

        # Loop through the path points and find distance between current and previous point
        for point in path:
            path_length += math.dist(previous_point, point)
            previous_point = point

        return path_length

    def show_path(self, map, path, scaling_factor=1):
        # Display the path on the occupancy map with image scaling for visualization
        display_map = map.copy()
        display_map = display_map.astype(np.uint8)
        display_map = cv2.cvtColor(display_map, cv2.COLOR_GRAY2RGB)

        # Add a point to the occupancy map with the path
        for x, y in path:
            display_map[y][x] = self.PATH_COLOUR
        cv2.imwrite("Path.png", display_map)
        # Display the map with the path
        # self.show_map(display_map, scaling_factor)

    def convert_to_pixel_point(self, real_point):
        # Convert pixel coordinate path to real [meters] coordinate
        origin_x, origin_y, origin_theta = self.map_origin
        real_x, real_y = real_point
        rotated_real_x, rotated_real_y= [real_x*math.cos(origin_theta) - real_y*math.sin(origin_theta), 
                                         real_x*math.sin(origin_theta) + real_y*math.cos(origin_theta)]
        # Convert to grid
        pixel_point = [round((rotated_real_x - origin_x) / self.map_resolution), round((rotated_real_y - origin_y) / self.map_resolution)]
        # Convert to grid map axis
        pixel_point = pixel_point[::-1] 
        pixel_point[0] = self.map.shape[0] - pixel_point[0] 
        return pixel_point

    def convert_to_real_point(self, pixel_point):
        # Convert pixel coordinate path to real [meters] coordinate
        origin_x, origin_y, origin_theta = self.map_origin
        pixel_x, pixel_y = pixel_point
        offset_pixel_x, offset_pixel_y = (pixel_x * self.map_resolution + origin_x, pixel_y * self.map_resolution + origin_y)
        real_point = [offset_pixel_x*math.cos(-origin_theta) - offset_pixel_y*math.sin(-origin_theta), 
                      offset_pixel_x*math.sin(-origin_theta) + offset_pixel_y*math.cos(-origin_theta)]
        return real_point

    def convert_to_real_path(self, path):
        # Convert path pixel coordinate path to real [meters] coordinate
        real_path = [self.convert_to_real_point(pixel) for pixel in path]
        return real_path

if __name__ == "__main__":
    try:
        Planning()
    except rospy.ROSInterruptException:
        pass
