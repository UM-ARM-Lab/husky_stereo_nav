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
from sklearn.cluster import KMeans


class Visualizer:
    def __init__(self):
        # rospy.Subscriber("fake_hose_cloud", PointCloud2, self.pc_callback)
        # rospy.Subscriber("zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.pc_callback)
        # rospy.Subscriber("rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.num_publishers = 3
        self.pc_pubs = [rospy.Publisher(f"filtered_pc{i}", PointCloud2, queue_size=1) for i in range(self.num_publishers)]
        self.visualize_pc_frame("data/zed_pc_frame1.txt")
        # self.visualize_pc_frame("data/fake_pc_frame1.txt")
    
    def visualize_pc_frame(self, pc_frame: str):
        points = np.loadtxt(pc_frame)
        points = points[points[:, 0] != np.inf]
        colors = points[:, 3:]

        self.fig = plt.figure()
        self.plot_rgb_space(colors, colors, r=1, c=2, i=1)
        # self.plot_xyz_space(points[:, :3], color_points=points[:, 3:])
        # self.plot_kmeans(points[:, 3:])
        
        distances = self.color_distance_grouping(colors)
        self.plot_rgb_space(colors, distances, r=1, c=2, i=2)
        plt.show()
        # self.publish_kmeans(points)
        

    def pc_callback(self, msg):
        print("callback")
        # Nx6 array of [x, y, z, r, g, b]
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        rgb_points = np.vstack([float_to_rgb(intensity) for intensity in points[:, 3]]) / 255
        points = np.column_stack((points[:, :3], rgb_points))
        # self.plot_histogram(points)
        # self.plot_rgb_space(points)
        self.plot_kmeans()

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

    def plot_rgb_space(self, points, colors, r=1, c=1, i=1):
        # Nx3 array of [r, g, b]
        # rgb_points = points[:, 3:]
        ax = self.fig.add_subplot(r, c, i, projection="3d")
        ax.clear()
        sc = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors,  cmap="plasma", marker="x")
        ax.set_xlabel("Red")
        ax.set_ylabel("Green")
        ax.set_zlabel("Blue")
        plt.colorbar(sc, label="color distance")
        # plt.pause(0.05)
    
    def kmeans(self, points: np.ndarray, n_clusters: int) -> np.ndarray:
        estimator = KMeans(n_clusters=n_clusters, random_state=0).fit(points)
        point_labels = estimator.predict(points)
        return point_labels
    
    def plot_kmeans(self, points):
        point_labels = self.kmeans(points, 2)
        group1 = points[point_labels == 0]
        group2 = points[point_labels == 1]

        ps = (points, group1, group2)
        titles = ("original", "group 1", "group 2")
        colors = (points, "r", "b")
        for i, (points, title, color) in enumerate(zip(ps, titles, colors)):
            ax = self.fig.add_subplot(1, 3, i + 1, projection="3d")
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, marker=".")
            ax.set_title(title)
            ax.set_xlabel("Red")
            ax.set_ylabel("Green")
            ax.set_zlabel("Blue")
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.set_zlim(0, 1)
    
    def publish_kmeans(self, points):
        point_labels = self.kmeans(points[:, 3:], 2)
        group1 = points[point_labels == 0]
        group2 = points[point_labels == 1]
        pointclouds = (points, group1, group2)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_pointclouds(pointclouds, frame_id="zed2i_left_camera_frame")
    
    def publish_pointclouds(self, pointclouds, frame_id="map"):
        for i, points in enumerate(pointclouds):
            header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
            fields = [
                PointField(name="x", offset=0, datatype=7, count=1),
                PointField(name="y", offset=4, datatype=7, count=1),
                PointField(name="z", offset=8, datatype=7, count=1),
                PointField(name="r", offset=12, datatype=7, count=1),
                PointField(name="g", offset=16, datatype=7, count=1),
                PointField(name="b", offset=20, datatype=7, count=1),
            ]
            self.pc_pubs[i].publish(pc2.create_cloud(header, fields, points))
        

    
    def plot_xyz_space(self, points, color_points):
        ax = self.fig.add_subplot(projection="3d")
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color_points, marker=".")
        ax.set_title("cartesian")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
         
    def get_avg_color(self, colors: np.ndarray) -> np.ndarray:
        return np.mean(colors, axis=0)
    
    def get_dominant_color(self, colors: np.ndarray, n_clusters) -> np.ndarray:
        """
        :param colors: nx3 numpy array containing RGB values for each point in the pointcloud
        """

        kmeans = KMeans(n_clusters=n_clusters, random_state=0).fit(colors)
        dominant_colors = kmeans.cluster_centers_
        print(f"dominant_colors array: {dominant_colors}")
        for color in dominant_colors:
            r, g, b = color
            print(f"({r}, {g}, {b})")
        labeled_colors = kmeans.predict(colors)
        # (108, 95, 79)
        # (64, 56, 50)

        # TODO: make this code better, no hardcoded cluster num
        if np.count_nonzero(labeled_colors == 0) >= np.count_nonzero(labeled_colors == 1):
            return dominant_colors[0]
        return dominant_colors[1]

    def color_distance_grouping(self, colors):
        floor_color = self.get_dominant_color(colors, n_clusters=2)
        color_distances = np.linalg.norm(colors - floor_color, axis=1)
        return color_distances
    
        
def main():
    rospy.init_node("colorspace_visualizer")
    vis = Visualizer()
    # rospy.spin()


if __name__ == "__main__":
    main()
