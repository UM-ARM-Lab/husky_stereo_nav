#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from arc_utilities.tf2wrapper import TF2Wrapper
import numpy as np
from color_utils import float_to_rgb
import tf2_ros
import sklearn.cluster

COLOR_THRESHOLD = 50

class RGBFilter:
    pc_pub: rospy.Publisher
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    
    def __init__(self):
        rospy.Subscriber("rtabmap/cloud_ground", PointCloud2, self.ground_cloud_callback)
        self.pc_pub = rospy.Publisher("filtered_ground", PointCloud2)
        self.tfw = TF2Wrapper()

    @staticmethod
    def get_avg_color(colors: np.ndarray) -> np.ndarray:
        return np.mean(colors, axis=0)
    
    def get_dominant_color(colors: np.ndarray) -> np.ndarray:
        """
        :param colors: nx3 numpy array containing RGB values for each point in the pointcloud
        """

        kmeans = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit_predict(colors)
        dominant_colors = kmeans.cluster_centers_
        print(f"dominant_colors array: {dominant_colors}")
        for color in dominant_colors:
            r, g, b = color
            print(f"({r}, {g}, {b})")
        labeled_colors = kmeans.labels_

        # TODO: make this code better, no hardcoded cluster num
        if np.count_nonzero(labeled_colors == 0) >= np.count_nonzero(labeled_colors == 1):
            return dominant_colors[0]
        return dominant_colors[1]

    def ground_cloud_callback(self, msg: PointCloud2):
        
        # read pointcloud from ROS message into a numpy array
        cloud_array = np.array(list(pc2.read_points(msg, skip_nans=True)))

        # TODO: make sure this is the right camera frame
        # map_to_camera_tf = self.tfw.get_transform(parent="map", child="zed2i_left_camera_frame")

        # convert pointcloud points to homogenous points and transform them into the map frame
        # cloud_array_in_cam_hom = np.vstack(cloud_array_in_cam, np.ones((1, cloud_array.shape[1])))
        # cloud_array = map_to_camera_tf @ cloud_array_in_cam_hom

        # extract the [x, y, z] points from the pointcloud
        # nx3 array
        points = cloud_array[:, :3]

        # extract the [r, g, b] colors from the pointcloud, converting them from float format
        # nx3 array
        colors = np.vstack([float_to_rgb(float_color) for float_color in cloud_array[:, 3]])

        # floor_color = self.get_avg_color(colors)
        floor_color = self.get_dominant_color(colors)

        # compute color deviation by subtracting the average color from each point and then taking the norm across r,g,b
        color_dev = np.linalg.norm(colors - floor_color, axis=1)

        # get the indices of outlier points where the color deviation is > a threshold,
        # returns a tuple but we only want the first element since its a 1D array
        outlier_ids = np.nonzero(color_dev > COLOR_THRESHOLD)[0]

        # select the outliers from the original pointcloud array
        color_outliers = cloud_array[outlier_ids, :]

        # outlier_cloud = array_to_pointcloud2(outlier_array, msg.header.stamp, msg.header.frame_id)
        outlier_cloud = pc2.create_cloud(msg.header, msg.fields, color_outliers)
        self.pc_pub.publish(outlier_cloud)



def main():
    rospy.init_node("rgb_filter")
    filter = RGBFilter()

    rospy.spin()
    
if __name__ == "__main__":
    main()