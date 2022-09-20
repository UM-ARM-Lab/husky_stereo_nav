import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from color_utils import float_to_rgb
from ros_numpy.point_cloud2 import array_to_pointcloud2

COLOR_THRESHOLD = 20

class RGBFilter:
    pc_pub: rospy.Publisher
    
    def __init__(self):
        rospy.Subscriber("rtabmap/cloud_ground", PointCloud2, self.ground_cloud_callback)
        self.pc_pub = rospy.Publisher("filtered_ground", PointCloud2)

    def get_avg_color(colors: np.ndarray) -> np.ndarray:
        return np.mean(colors, axis=0)
    
    def get_dominant_color(colors: np.ndarray) -> np.ndarray:
        ...
        
    def ground_cloud_callback(self, msg: PointCloud2):
        
        # read pointcloud from ROS message into a numpy array
        cloud_array = np.array(list(pc2.read_points(msg, skip_nans=True)))

        # extract the [x, y, z] points from the pointcloud
        # nx3 array
        points = cloud_array[:, :3]

        # extract the [r, g, b] colors from the pointcloud, converting them from float format
        # nx3 array
        colors = np.vstack([float_to_rgb(float_color) for float_color in cloud_array[:, 3]])

        floor_color = self.get_avg_color(colors)

        # compute color deviation by subtracting the average color from each point and then taking the norm across r,g,b
        color_dev = np.linalg.norm(colors - floor_color, axis=1)

        # get the indices of outlier points where the color deviation is > a threshold,
        # returns a tuple but we only want the first element since its a 1D array
        outlier_ids = np.nonzero(color_dev > COLOR_THRESHOLD)[0]

        # select the outliers from the original pointcloud array
        outlier_array = cloud_array[outlier_ids, :]

        outlier_cloud = array_to_pointcloud2(outlier_array, msg.header.stamp, msg.header.frame_id)
        self.pc_pub.publish(outlier_cloud)


def main():
    rospy.init_node("rgb_filter")
    filter = RGBFilter()
    
if __name__ == "__main__":
    main()