#!/usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import cv2
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from color_utils import float_to_rgb


class Visualizer:
    def __init__(self):
        # rospy.Subscriber("fake_hose_cloud", PointCloud2, self.pc_callback)
        rospy.Subscriber("zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.pc_callback)
        # rospy.Subscriber("rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        # plt.ion()

    def pc_callback(self, msg):
        print("callback")
        # Nx6 array of [x, y, z, r, g, b]
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        rgb_points = np.vstack([float_to_rgb(intensity) for intensity in points[:, 3]]) / 255
        points = np.column_stack((points[:, :3], rgb_points))
        self.plot_histogram(points)

    def plot_histogram(self, points):
        red_vals = points[:, 3]
        f, axs = plt.subplots(1, 3, sharey=True)
        axs[0].hist(red_vals, bins=50)
        axs[0].set_ylabel("Frequency")
        axs[0].set_xlabel("Red Intensity")
        axs[1].hist(points[:, 4], bins=50)
        # axs[1].set_ylabel("Frequency")
        axs[1].set_xlabel("Green Intensity")
        axs[2].hist(points[:, 5], bins=50)
        # axs[2].set_ylabel("Frequency")
        axs[2].set_xlabel("Blue Intensity")
        plt.show()

    def plot_rgb_space(self, points):
        # Nx3 array of [r, g, b]
        # rgb_points = points[:, 3:]
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(projection="3d")
        self.ax.clear()
        self.ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points, marker="x")
        self.ax.set_xlabel("Red")
        self.ax.set_ylabel("Green")
        self.ax.set_zlabel("Blue")
        # plt.pause(0.05)
        plt.show(block=True)
        
def main():
    rospy.init_node("colorspace_visualizer")
    filter = Visualizer()
    rospy.spin()


if __name__ == "__main__":
    main()
