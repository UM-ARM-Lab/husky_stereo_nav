#!/usr/bin/python3
from turtle import color, distance

from matplotlib.colors import rgb_to_hsv
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import sklearn.cluster
# from colorsys import rgb_to_hsv
from matplotlib.colors import rgb_to_hsv


class KMeans:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.pc_callback)
        self.cloud_pub1 = rospy.Publisher("/cloud_ground_hue", PointCloud2, queue_size=10)
        self.cloud_pub2 = rospy.Publisher("/zed_hue", PointCloud2, queue_size=10)

    def pc_callback(self, msg: PointCloud2):
        print(f"frame: {msg.header.frame_id}\n\n")
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        float_rgb = np.vstack([float_to_rgb(intensity) for intensity in points[:, 3]]) / 255
        hsv = rgb_to_hsv(float_rgb)
        distance_fields = np.column_stack((10*points[:, 2], 100*hsv[:, 0]))
        distances = np.linalg.norm(distance_fields, axis=1)
        rgb_norm = np.linalg.norm(float_rgb, axis=1)

        hsv_points = np.column_stack((points, hsv, distances, rgb_norm))

        hue_fields = msg.fields
        hue_fields.append(PointField(name="h", offset=20, datatype=7, count=1))
        hue_fields.append(PointField(name="s", offset=24, datatype=7, count=1))
        hue_fields.append(PointField(name="v", offset=28, datatype=7, count=1))
        hue_fields.append(PointField(name="d", offset=32, datatype=7, count=1))
        hue_fields.append(PointField(name="rgb_norm", offset=36, datatype=7, count=1))
        # print(f"fields: {hue_fields}")
        hue_cloud = pc2.create_cloud(msg.header, hue_fields, hsv_points)
        if msg.header.frame_id == "zed2i_left_camera_frame":
            self.cloud_pub2.publish(hue_cloud)
        else:
            self.cloud_pub1.publish(hue_cloud)


def main():
    rospy.init_node("kmeans_filter")
    filter = KMeans();
    rospy.spin()

if __name__ == "__main__":
    main()