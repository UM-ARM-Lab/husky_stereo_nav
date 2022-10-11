#!/usr/bin/python3
from matplotlib.colors import rgb_to_hsv
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
# import sklearn.cluster
# from colorsys import rgb_to_hsv
from matplotlib.colors import rgb_to_hsv


class Otsus:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        # rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.pc_callback)
        self.cloud_pub1 = rospy.Publisher("/cloud_ground_hue", PointCloud2, queue_size=10)
        # self.cloud_pub2 = rospy.Publisher("/zed_hue", PointCloud2, queue_size=10)

    def pc_callback(self, msg: PointCloud2):
        # print(f"frame: {msg.header.frame_id}\n\n")
        # read point cloud into a Nx4 np array for [x, y, z, color]
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))

        # convert intensity color vector to a Nx3 np array of [r, g, b], scaled as a float from [0, 1]
        float_rgb = np.vstack([float_to_rgb(intensity) for intensity in points[:, 3]]) / 255
        
        # convert rgb to Nx3 array of [h, s, v] all in range of [0, 1]
        hsv = rgb_to_hsv(float_rgb)

        # horizontally stack all column vectors of data we want to use for the overall cost metric
        z = points[:, 2]
        h = hsv[:, 0]
        cost_fields = np.column_stack((z, float_rgb))

        # compute the norm to get single value cost for each point
        costs = np.linalg.norm(cost_fields, axis=1)
        
        # create an Nx3 array of [x, y, cost]
        cost_points = np.column_stack((points[:, :2], costs))
        obstacle_ids = self.otsus(cost_points)
        rgb_norm = np.linalg.norm(float_rgb, axis=1)

        # put all the new data columns on the end of the original point cloud matrix
        extra_data_points = np.column_stack((points, hsv, costs, rgb_norm))

        # add fields corresponding to the new data columns added
        fields = msg.fields
        fields.append(PointField(name="h", offset=20, datatype=7, count=1))
        fields.append(PointField(name="s", offset=24, datatype=7, count=1))
        fields.append(PointField(name="v", offset=28, datatype=7, count=1))
        fields.append(PointField(name="d", offset=32, datatype=7, count=1))
        fields.append(PointField(name="rgb_norm", offset=36, datatype=7, count=1))
        # print(f"fields: {fields}")
        # print(f"pointcloud is {msg.height} by {msg.width}")

        # convert point cloud matrix back to ROS point cloud type then publish it
        hue_cloud = pc2.create_cloud(msg.header, fields, extra_data_points)
        # if msg.header.frame_id == "zed2i_left_camera_frame":
        #     self.cloud_pub2.publish(hue_cloud)
        # else:
        # print(f"new pc is {hue_cloud.height} by {hue_cloud.width}")
        self.cloud_pub1.publish(hue_cloud)

    def otsus(cost_points: np.ndarray) -> np.ndarray:
        
        # find min and max x and y
        ...
        
        # create an opencv matrix of size equal to some multiple of the pointcloud range
        
        # for each point in the cloud
            # divide its x and y by something to get its index in the image matrix
            # if the pixel at that spot is empty, assign the cost to it (mapped to 255)
            # if the pixel is not empty, average it ???

        # run otsus method on the opencv matrix


def main():
    rospy.init_node("kmeans_filter")
    filter = Otsus();
    rospy.spin()

if __name__ == "__main__":
    main()