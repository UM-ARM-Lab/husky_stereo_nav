#!/usr/bin/python3
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sklearn.cluster

RGB_SCALE = 0;

class KMeans:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.cloud_pub0 = rospy.Publisher("/kmeans_ground_filtered0", PointCloud2, queue_size=1)
        self.cloud_pub1 = rospy.Publisher("/kmeans_ground_filtered1", PointCloud2, queue_size=1)

    def pc_callback(self, msg: PointCloud2):
        print(f"frame: {msg.header.frame_id}\n\n")
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        # print(points)
        rgb = np.vstack(float_to_rgb(intensity) for intensity in points[:, 3])
        # print(points[:, -2])
        # print(rgb)
        rgb = RGB_SCALE * rgb
        rgb_points = np.column_stack((points[:, -2], rgb))
        # print(points)
        #rospy.logwarn(points);
        print(f"points before: {rgb_points}\n\n")
        kmeans = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit(points)
        # print(f"points: {points}\n\n")
        labels = kmeans.predict(points)
        # print(f"labels: {labels}\n\n")
        outlier_ids0 = np.nonzero(labels == 0)[0]
        outlier_ids1 = np.nonzero(labels == 1)[0]
        # print(f"outlier IDs: {outlier_ids}\n\n")
        #rospy.logwarn(outlier_ids)
        outliers0 = points[outlier_ids0, :]
        outliers1 = points[outlier_ids1, :]
        print(f"outliers: {outliers0}\n\n");
        # kmeans.labels_;
        #rospy.logwarn(labels)
        kmeans_cloud0 = pc2.create_cloud(msg.header, msg.fields, outliers0)
        kmeans_cloud1 = pc2.create_cloud(msg.header, msg.fields, outliers1)
        self.cloud_pub0.publish(kmeans_cloud0)
        self.cloud_pub1.publish(kmeans_cloud1)

def main():
    rospy.init_node("kmeans_filter")
    filter = KMeans();
    rospy.spin()

if __name__ == "__main__":
    main()