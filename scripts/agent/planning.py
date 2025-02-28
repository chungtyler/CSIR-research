import rospy
import cv2
import numpy as np
import yaml
import math
from PIL import Image
import pyastar2d as astar

class Planning:
    '''
    Path planning class handling waypoint creation and navigatio to each waypoint
    '''
    def __init__(self, path_to_config):
        # Initialize occupancy map and path planning
        self.map, self.map_resolution, self.map_origin = self.load_map_config(path_to_config / 'map/map.pgm', path_to_config / 'map/map.yaml')
        self.PATH_COLOUR = (255, 0, 255)

    def load_map_config(self, map_pgm_path, map_yaml_path):
        # Get map.pgm and map.yaml to create binary map with known resolution and origin
        map_pgm = Image.open(map_pgm_path)
        map = np.array(map_pgm)

        # Convert map.pgm to binary occupancy grid
        # map = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
        # map = cv2.imread(map_pgm, cv2.IMREAD_GRAYSCALE)
        map = np.where(map == 0, np.inf, 1.0)

        # Read and record map resolution [meters / pixel] and origin
        with open(map_yaml_path, 'r') as map_yaml:
            map_data = yaml.safe_load(map_yaml)

        map_resolution = map_data['resolution']
        map_origin = map_data['origin']

        return map, map_resolution, map_origin

    def pad_map(self, map, radius):
        # Pad the occupied areas of the map with defined radius to prevent agent collision
        padded_map = map.copy()
        for rows in range(len(padded_map)):
            for cols in range(len(padded_map[0])):
                if map[rows][cols] == 0:
                    cv2.circle(padded_map, [cols, rows], radius, 1, -1)
        return padded_map

    def show_map(self, map, scaling_factor=1):
        # Display the map and resize it accordingle for visualization
        map_shape = map.shape
        map_shape_resized = (round(map_shape[1]*scaling_factor), round(map_shape[0]*scaling_factor)) # Resize aspect ratio or shape of the map
        map_resized = cv2.resize(map, map_shape_resized, interpolation=cv2.INTER_CUBIC) # Apply scaling to the map

        # Display map
        cv2.imshow("Occupancy Map", map_resized)
        cv2.waitKey(-1)
        cv2.destroyAllWindows

    def generate_path(self, start, goal):
        # Get AStar path as a list of tuples (x, y)
        path = astar.astar_path(self.map, start, goal)
        return path

    def get_path_length(self, path):
        # return path_length by finding the distance between points and summing the total length
        path_length = 0
        previous_x, previous_y = path[0]

        # Loop through the path points and find distance between current and previous point
        for x, y in path:
            path_length += math.dist(previous_x - x, previous_y - y)
            previous_x, previous_y = (x, y)

        return path_length

    def show_path(self, map, path, scaling_factor=1):
        # Display the path on the occupancy map with image scaling for visualization
        display_map = map.copy()

        # Add a point to the occupancy map with the path
        for x, y in path:
            display_map[y][x] = self.PATH_COLOUR

        # Display the map with the path
        self.show_map(display_map, scaling_factor)

    def convert_to_pixel_point(self, real_point):
        # Convert pixel coordinate path to real [meters] coordinate
        pixel_point = ((real_point[0] - self.map_origin[0]) / self.map_resolution, (real_point[1] - self.map_origin[1]) /  self.map_resolution)
        return pixel_point

    def convert_to_real_point(self, pixel_point):
        # Convert pixel coordinate path to real [meters] coordinate
        real_point = (pixel_point[0] * self.map_resolution + self.map_origin[0], pixel_point[1] * self.map_resolution + self.map_origin[1])
        return real_point

    def convert_to_real_path(self, path):
        # Convert path pixel coordinate path to real [meters] coordinate
        real_path = [self.real_point(pixel) for pixel in path]
        return real_path

if __name__ == "__main__":
    try:
        Planning()
    except rospy.ROSInterruptException:
        pass
