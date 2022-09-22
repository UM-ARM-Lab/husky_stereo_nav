#!/usr/bin/python3
from msilib.schema import PublishComponent
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import KMeans

class KMeans:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        self.cloud_pub = rospy.Publisher("/kmeans_ground_filtered", PointCloud2)
        rospy.init_node("kmeans_ground")
        rospy.spin()

    def pc_callback(self, ground_cloud, msg):
        points = np.array(list(pc2.read_points(ground_cloud, skip_nans=True)))
        rgb = np.vstack(float_to_rgb(intensity) for intensity in points[:, 3])
        points = np.hstack((points[:,[0,1,2]], rgb))
        #rospy.logwarn(points);
        kmeans = KMeans(n_clusters=2, random_state=0).fit(points)
        rospy.logwarn(points)
        labels = kmeans.predict(points)
        outlier_ids = np.nonzero(labels == 1)
        outliers = points[outlier_ids, :]
        # kmeans.labels_;
        rospy.logwarn(labels)
        points1 = kmeans.labels_
        pc2.create_cloud(msg.header, msg.fields, outliers)
        self.cloud_pub.publish(outliers)

if __name__ == "__main__":
    main()