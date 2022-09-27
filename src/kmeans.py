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

RGB_SCALE = 1/255;
HSV_SCALE = 1/360;

class KMeans:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.cloud_pub0 = rospy.Publisher("/kmeans_rgb_filtered", PointCloud2, queue_size=1)
        self.cloud_pub1 = rospy.Publisher("/kmeans_hsv_filtered", PointCloud2, queue_size=1)

    def pc_callback(self, msg: PointCloud2):
        print(f"frame: {msg.header.frame_id}\n\n")
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        rgb = np.vstack(float_to_rgb(intensity) for intensity in points[:, 3])
        hsv = convertToHSV(rgb)
        rgb = RGB_SCALE * rgb
        hsv = HSV_SCALE * hsv
        rgb_points = np.column_stack((points[:, -2], rgb))
        hsv_points = np.column_stack((points[:, -2], hsv))
        kmeans_rgb = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit(rgb_points)
        kmeans_hsv = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit(hsv_points)
        labels_rgb = kmeans_rgb.predict(rgb_points)
        labels_hsv = kmeans_hsv.predict(hsv_points)
        outlier_ids_rgb = np.nonzero(labels_rgb == 0)[0]
        outlier_ids_hsv = np.nonzero(labels_hsv == 0)[0]
        outliers_rgb = points[outlier_ids_rgb, :]
        outliers_hsv = points[outlier_ids_hsv, :]
        rgb_cloud = pc2.create_cloud(msg.header, msg.fields, outliers_rgb)
        hsv_cloud = pc2.create_cloud(msg.header, msg.fields, outliers_hsv)
        self.cloud_pub0.publish(rgb_cloud)
        self.cloud_pub1.publish(hsv_cloud)

        # kmeans = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit(rgb_points)
        # labels = kmeans.predict(rgb_points)
        # outlier_ids0 = np.nonzero(labels == 0)[0]
        # outliers0 = points[outlier_ids0, :]
        # print(f"outliers: {outliers0}\n\n");
        # kmeans_cloud0 = pc2.create_cloud(msg.header, msg.fields, outliers0)
        # self.cloud_pub0.publish(kmeans_cloud0)

#rgb = ([r, g, b])
def convertToHSV(rgb):
    hsv = np.array(rgb, copy=True)
    for i in rgb[1]:
        hsv[i] = colorsys.rgb_to_hsv(*rgb[i])
    return hsv

def main():
    rospy.init_node("kmeans_filter")
    filter = KMeans();
    rospy.spin()

if __name__ == "__main__":
    main()