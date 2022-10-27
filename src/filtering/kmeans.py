#!/usr/bin/python3
from turtle import color

from matplotlib.colors import rgb_to_hsv
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sklearn.cluster
import colorsys

RGB_SCALE = 1 / 255
HSV_SCALE = 1 / 800


class KMeans:
    # cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.cloud_pub0 = rospy.Publisher(
            "/kmeans_rgb_filtered", PointCloud2, queue_size=1
        )
        self.cloud_pub1 = rospy.Publisher(
            "/kmeans_hsv_filtered", PointCloud2, queue_size=1
        )

    def pc_callback(self, msg: PointCloud2):
        print(f"frame: {msg.header.frame_id}\n\n")
        hsv_cloud = publish_hsv_filter(msg)
        # self.cloud_pub0.publish(rgb_cloud)
        self.cloud_pub1.publish(hsv_cloud)


def convertToHSV(rgb):
    hsv = np.array(rgb, copy=True)
    for i in rgb:
        hsv[i] = colorsys.rgb_to_hsv(*i)
    return hsv


def publish_hsv_filter(msg):
    points = np.array(list(pc2.read_points(msg, skip_nans=True)))
    rgb = np.vstack(float_to_rgb(intensity) for intensity in points[:, 3])
    hsv = convertToHSV(rgb)
    hsv = HSV_SCALE * hsv

    # for looking at h only
    color_points = hsv[:, 0]
    color_points = color_points.reshape(-1, 1)

    # zh
    # color_points = np.column_stack((hsv[:, 0]))

    kmeans_color = sklearn.cluster.KMeans(n_clusters=5, random_state=0).fit(
        color_points
    )
    labels_color = kmeans_color.predict(color_points)
    outlier_ids_color = np.nonzero(labels_color == 1)[0]
    outliers_color = points[outlier_ids_color, :]

    # list of hues of points in cluster 1
    hue_list = hsv[outlier_ids_color]
    # average hue in cluster 1
    print(f"\n\nAVERAGE 0: {np.mean(hue_list, axis=0)[0] * 200}\n\n")
    color_cloud = pc2.create_cloud(msg.header, msg.fields, outliers_color)
    return color_cloud


def main():
    rospy.init_node("kmeans_filter")
    filter = KMeans()
    rospy.spin()


if __name__ == "__main__":
    main()
