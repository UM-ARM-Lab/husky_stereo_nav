#!/usr/bin/python3
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import sklearn.cluster

class KMeans:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.cloud_pub = rospy.Publisher("/kmeans_ground_filtered", PointCloud2, queue_size=1)

    def pc_callback(self, msg: PointCloud2):
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))
        # print(points)
        rgb = np.vstack(float_to_rgb(intensity) for intensity in points[:, 3])
        rgb_points = np.hstack((points[:,:3], rgb))
        # print(points)
        #rospy.logwarn(points);
        print(f"points before: {points}\n\n")
        kmeans = sklearn.cluster.KMeans(n_clusters=2, random_state=0).fit(points)
        print(f"points: {points}\n\n")
        labels = kmeans.predict(points)
        print(f"labels: {labels}\n\n")
        outlier_ids = np.nonzero(labels == 1)[0]
        print(f"outlier IDs: {outlier_ids}\n\n")
        #rospy.logwarn(outlier_ids)
        outliers = points[outlier_ids, :]
        print(f"outliers: {outliers}\n\n");
        # kmeans.labels_;
        #rospy.logwarn(labels)
        kmeans_cloud = pc2.create_cloud(msg.header, msg.fields, outliers)
        self.cloud_pub.publish(kmeans_cloud)

def main():
    rospy.init_node("kmeans_filter")
    filter = KMeans();
    rospy.spin()

if __name__ == "__main__":
    main()