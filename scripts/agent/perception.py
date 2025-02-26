import rospy
import cv2
import json
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan, PointCloud2
import numpy as np

class Perception:
    '''
    This node subscribes to the agents sensors to collect sensor data and standardizes the formats
    This includes recieving the RealSense camera RGB image and depth image as well as the Velodyne-16 LiDAR's 2D and 3D scans
    '''
    def __init__(self, path_to_config, MIN_DEPTH_RANGE=0, MAX_DEPTH_RANGE=2000):
<<<<<<< HEAD
        # rospy.init_node('perception_node', anonymous=True)

=======
>>>>>>> main
        # Load rostopic names
        with open(path_to_config + '/rostopics.json', 'r') as rostopics_file:
            rostopics = json.load(rostopics_file)

        self.sub_rgb_image = rospy.Subscriber(rostopics['rgb_image'], Image, self.rgb_image_callback, queue_size=1)
        self.sub_depth_image = rospy.Subscriber(rostopics['depth_image'], Image, self.depth_image_callback, queue_size=1)
        self.sub_scan = rospy.Subscriber(rostopics['scan'], LaserScan, self.scan_callback, queue_size=1)
        self.sub_point_cloud = rospy.Subscriber(rostopics['point_cloud'], PointCloud2, self.point_cloud_callback, queue_size=1)

        self.bridge = CvBridge()

        self.rgb_image = None
        self.depth_image = None
        self.scan = None
        self.point_cloud = None

        self.MIN_DEPTH_RANGE = MIN_DEPTH_RANGE
        self.MAX_DEPTH_RANGE = MAX_DEPTH_RANGE

    def rgb_image_callback(self, data):
        # Get camera RGB image
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            rospy.logerr(f"RGB image processing failed: {e}")

    def depth_image_callback(self, data):
        # Get camera depth image
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, '16UC1') # Convert to 16 bit format
            depth_image = np.clip(depth_image, self.MIN_DEPTH_RANGE, self.MAX_DEPTH_RANGE) # Clip out depth values
            depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX) # Normalize data points
            depth_image = cv2.convertScaleAbs(depth_image) # Convert to 8-bit image
            self.depth_image = depth_image
        except Exception as e:
            rospy.logerr(f"Depth image processing failed: {e}")

    def scan_callback(self, data):
        # Get LiDAR 2D scan
        self.scan = data

    def point_cloud_callback(self, data):
        # Get LiDAR 3D point cloud
        self.point_cloud = data

    def show_image(self, image):
        # Show image for visualization
        cv2.imshow("Image", image)
        cv2.waitKey(-1)
        cv2.destroyAllWindows

    def get_object_data(self, image, object='person'):
        # TODO using yolov5 get the object information from the image such as X,Y,confidence, etc
        pass

    def crop_to_object(self, image, object_data, padding=25):
        # TODO crop the image of the object to its bounding box with some padding value
        # return the cropped image
        pass

if __name__ == "__main__":
    try:
        Perception()
    except rospy.ROSInterruptException:
        pass
<<<<<<< HEAD
    
=======
>>>>>>> main
