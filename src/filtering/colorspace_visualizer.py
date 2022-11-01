#!/usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.colors import rgb_to_hsv
import utils
from sklearn.cluster import KMeans
import plotly.graph_objects as go


class Visualizer:
    def __init__(self):
        # rospy.Subscriber("fake_hose_cloud", PointCloud2, self.pc_callback)
        # rospy.Subscriber("zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.pc_callback)
        # rospy.Subscriber("rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.num_publishers = 3
        self.pc_pubs = [
            rospy.Publisher(f"filtered_pc{i}", PointCloud2, queue_size=1)
            for i in range(self.num_publishers)
        ]
        # self.visualize_pc_frame("data/zed_pc_frame1.txt")
        self.visualize()
        # self.visualize_pc_frame("data/fake_pc_frame1.txt")

    def visualize(self):
        rgb_colors = utils.load_img_frame("data/rgb_img_zed1.png", blur=True)
        hsv_colors = rgb_to_hsv(rgb_colors)
        labels = utils.load_img_labels("data/labeled_hose_ground_zed.png")
        color_strings = [f"rgb({r},{g},{b})" for r, g, b in rgb_colors]

        hose_rgbs = rgb_colors[labels == 0.5]
        floor_rgbs = rgb_colors[labels == 1]
        other_rgbs = rgb_colors[labels == 0]

        hose_hsvs = hsv_colors[labels == 0.5]
        floor_hsvs = hsv_colors[labels == 1]
        other_hsvs = hsv_colors[labels == 0]
        # self.plot_histogram(rgb_colors, hsv_colors, title="All Points")
        # self.plot_histogram(floor_rgbs, floor_hsvs, title="Floor")
        # self.plot_histogram(hose_rgbs, hose_hsvs, title="Hose")
        # self.plot_histogram(other_rgbs, other_hsvs, title="Other")
        f, axs = plt.subplots(1, 3, sharey=True)
        axs[0].hist(hsv_colors[:, 0], bins=100, color="c")
        axs[0].set_xlabel("Hue")
        axs[1].hist(hsv_colors[:, 1], bins=100, color="m")
        axs[1].set_xlabel("Saturation")
        axs[2].hist(hsv_colors[:, 2], bins=100, color="y")
        axs[2].set_xlabel("Value")
        plt.suptitle("All Points")
        for ax in axs:
            ax.set_xlim(0, 1)
            # ax.set_ylim(0, 20000)

        f, axs = plt.subplots(1, 3, sharey=True)
        axs[0].hist(floor_hsvs[:, 0], bins=100, color="c")
        axs[0].set_xlabel("Hue")
        axs[1].hist(floor_hsvs[:, 1], bins=100, color="m")
        axs[1].set_xlabel("Saturation")
        axs[2].hist(floor_hsvs[:, 2], bins=100, color="y")
        axs[2].set_xlabel("Value")
        plt.suptitle("Floor")
        for ax in axs:
            ax.set_xlim(0, 1)
            # ax.set_ylim(0, 20000)

        f, axs = plt.subplots(1, 3, sharey=True)
        axs[0].hist(other_hsvs[:, 0], bins=100, color="c")
        axs[0].set_xlabel("Hue")
        axs[1].hist(other_hsvs[:, 1], bins=100, color="m")
        axs[1].set_xlabel("Saturation")
        axs[2].hist(other_hsvs[:, 2], bins=100, color="y")
        axs[2].set_xlabel("Value")
        plt.suptitle("Other")
        for ax in axs:
            ax.set_xlim(0, 1)
            # ax.set_ylim(0, 20000)
        plt.show()

        # obstacle_labels = self.histogram_filter(hsv_colors, floor_hsvs)
        # img = cv2.imread("data/rgb_img_zed1.png")
        # obstacle_labels = obstacle_labels.reshape((img.shape[0], img.shape[1]))
        # img[obstacle_labels] = np.array([255, 0, 255])
        # cv2.imwrite("data/segmented_rgb2.png", img)
        # cv2.imshow("labeled", img)
        # cv2.waitKey(0)
       
        # points = utils.load_pc_frame("data/zed_pc_frame1.txt")
        # rgb_colors = points[:, 3:]

        # self.fig = plt.figure()
        # self.plot_rgb_space(colors, colors, r=1, c=2, i=1)
        # self.plot_rgb_space(colors, labels, r=1, c=2, i=2)
        # self.plot_xyz_space(points[:, :3], color_points=points[:, 3:])
        # self.plot_kmeans(points[:, 3:])

        # distances = self.color_distance_grouping(colors)
        # self.plot_rgb_space(colors, distances, r=1, c=2, i=2)
        # self.plot_hsv_space(hsv_colors, rgb_colors, r=1, c=2, i=1)
        # self.plot_hsv_space(hsv_colors, labels, r=1, c=2, i=2)
        # self.plot_hs(hsv_colors, rgb_colors, r=1, c=2, i=1)
        # self.plot_hs(hsv_colors, labels, r=1, c=2, i=2)
        # plt.show()
        # self.go_plot_rgb_space(rgb_colors, rgb_colors)
        # self.go_plot_rgb_space(rgb_colors, labels)
        # self.go_plot_hsv_space(hsv_colors, rgb_colors)
        # self.go_plot_hsv_space(hsv_colors, labels)
        # self.publish_kmeans(points)

    def pc_callback(self, msg):
        print("callback")
        # Nx6 array of [x, y, z, r, g, b]
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        rgb_points = (
            np.vstack([utils.float_to_rgb(intensity) for intensity in points[:, 3]])
            / 255
        )
        points = np.column_stack((points[:, :3], rgb_points))
        # self.plot_histogram(points)
        # self.plot_rgb_space(points)
        self.plot_kmeans()

    def plot_histogram(self, rgbs, hsvs, title=""):
        f, axs = plt.subplots(2, 3, sharey=True)
        f.suptitle(title)
        axs[0][0].hist(rgbs[:, 0], bins=50, color="r")
        axs[0][0].set_ylabel("Frequency")
        axs[0][0].set_xlabel("Red")
        axs[0][1].hist(rgbs[:, 1], bins=50, color="g")
        axs[0][1].set_xlabel("Green")
        axs[0][2].hist(rgbs[:, 2], bins=50, color="b")
        axs[0][2].set_xlabel("Blue")

        axs[1][0].hist(hsvs[:, 0], bins=50, color="c")
        axs[1][0].set_ylabel("Frequency")
        axs[1][0].set_xlabel("Hue")
        axs[1][1].hist(hsvs[:, 1], bins=50, color="m")
        axs[1][1].set_xlabel("Saturation")
        axs[1][2].hist(hsvs[:, 2], bins=50, color="y")
        axs[1][2].set_xlabel("Value")

        for row in axs:
            for ax in row:
                ax.set_xlim(0, 1)
                ax.set_ylim(0, 20000)

    def plot_hs(self, points, colors, r=1, c=1, i=1):
        ax = self.fig.add_subplot(r, c, i)
        sc = ax.scatter(points[:, 0], points[:, 1], c=colors, cmap="plasma", marker=".")
        ax.set_xlabel("Hue")
        ax.set_ylabel("Saturation")
        
    def plot_rgb_space(self, points, colors, r=1, c=1, i=1):
        # Nx3 array of [r, g, b]
        # rgb_points = points[:, 3:]
        ax = self.fig.add_subplot(r, c, i, projection="3d")
        ax.clear()
        sc = ax.scatter(
            points[:, 0],
            points[:, 1],
            points[:, 2],
            c=colors,
            cmap="plasma",
            marker=".",
        )
        ax.set_xlabel("Red")
        ax.set_ylabel("Green")
        ax.set_zlabel("Blue")
        # plt.colorbar(sc, label="color distance")
        # plt.pause(0.05)

    def plot_hsv_space(self, hsv_points, colors, r=1, c=1, i=1):
        ax = self.fig.add_subplot(r, c, i, projection="3d")
        sc = ax.scatter(
            hsv_points[:, 0],
            hsv_points[:, 1],
            hsv_points[:, 2],
            c=colors,
            cmap="plasma",
            marker=".",
        )
        ax.set_xlabel("Hue")
        ax.set_ylabel("Saturation")
        ax.set_zlabel("Value")
        # plt.colorbar(sc, label="color distance")
        # plt.pause(0.05)

    def go_plot_rgb_space(self, points, color_strings, r=1, c=1, i=1):
        # Nx3 array of [r, g, b]
        # rgb_points = points[:, 3:]
        sc = go.Scatter3d(
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            marker=go.scatter3d.Marker(color=color_strings, size=1),
            opacity=0.9,
            mode="markers",
            name="floor",
            showlegend=True
        )
        fig = go.Figure(data=sc)
        fig.update_layout(
            scene=dict(
                xaxis_title="Red",
                yaxis_title="Green",
                zaxis_title="Blue",
            ),
            width=700,
            margin=dict(r=20, b=10, l=10, t=10),
        )
        fig.show()

    def go_plot_hsv_space(self, hsv_points, color_strings, r=1, c=1, i=1):
        sc = go.Scatter3d(
            x=hsv_points[:, 0],
            y=hsv_points[:, 1],
            z=hsv_points[:, 2],
            marker=go.scatter3d.Marker(color=color_strings, size=1),
            opacity=0.9,
            mode="markers",
        )
        fig = go.Figure(data=sc)
        fig.update_layout(
            scene=dict(
                xaxis_title="Hue",
                yaxis_title="Saturation",
                zaxis_title="Value",
            ),
            width=700,
            margin=dict(r=20, b=10, l=10, t=10),
        )
        fig.show()

    def histogram_filter(self, hsvs, ref_hsvs):
        # make histograms of H and S or H and V for ref points
        # could be a 2D histogram
        h_hist, h_bins = np.histogram(ref_hsvs[:, 0], bins=20, density=True)
        s_hist, s_bins = np.histogram(ref_hsvs[:, 1], bins=20, density=True)
        v_hist, v_bins = np.histogram(ref_hsvs[:, 2], bins=20, density=True)

        # filter threshold out low H and S values
        
        # get values of each point in image in each histogram
        # if either histogram value is too low, its an obstacle
        # TODO: we cant just delete these, need to understand if these obstacles or not and then save that info
        # if a value is outside of the range of the histogram, it's definitely an obstacle because it has zero entries 
        h_ids = np.digitize(hsvs[:, 0], h_bins) - 1
        out_of_bounds_ids = (h_ids <= 0) | (h_ids >= 20)
        h_ids[out_of_bounds_ids] = 1
        s_ids = np.digitize(hsvs[:, 1], s_bins) - 1
        out_of_bounds_ids = (s_ids <= 0) | (s_ids >= 20)
        s_ids[out_of_bounds_ids] = 1
        # v_ids = np.digitize(hsvs[:, 2], v_bins) - 1
        # out_of_bounds_ids = (v_ids <= 0) | (v_ids >= 20)
        # v_ids[out_of_bounds_ids] = 1

        h_obstacles = h_hist[h_ids] < 0.5
        s_obstacles = s_hist[s_ids] < 0.2
        # v_obstacles = v_hist[v_ids] < 0.5

        # is_obstacle = h_obstacles | s_obstacles | v_obstacles | out_of_bounds_ids
        is_obstacle = h_obstacles | s_obstacles | out_of_bounds_ids
        # obstacle_labels = np.zeros(is_obstacle.shape[0])
        # obstacle_labels[is_obstacle] = 1
        return is_obstacle
        
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
        if np.count_nonzero(labeled_colors == 0) >= np.count_nonzero(
            labeled_colors == 1
        ):
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
