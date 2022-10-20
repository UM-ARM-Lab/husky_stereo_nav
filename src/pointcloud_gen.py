#!/usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import cv2


class Generator:
    def __init__(self):
        self.cloud_pub = rospy.Publisher("/fake_hose_cloud", PointCloud2, queue_size=10)

    def image_to_pointcloud(img: np.ndarray) -> PointCloud2:
        # convert color ordering from BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # constants for scale and offset of pointcloud grid
        pixel_size = 0.05
        x_offset = -5
        y_offset = -5

        # create an empty NxMx3 matrix to store 3D point coordinates
        coords = np.zeros(img.shape)

        # scale and shift the indices to create X and Y coordinates
        coords[:, :, 0] = (
            np.arange(img.shape[0]).reshape((1, -1)) * pixel_size
        ) + x_offset
        coords[:, :, 1] = (
            np.arange(img.shape[1]).reshape((-1, 1)) * pixel_size
        ) + y_offset

        # flatten and stack together the coordinates and the image to get a (N*M)x6 [x, y, z, r, g, b] matrix
        points = np.column_stack(
            (coords.reshape((-1, 3)), img.reshape((-1, 3)).astype("float64") / 255.0)
        )

        header = Header(stamp=rospy.Time.now(), frame_id="map")
        fields = [
            PointField(name="x", offset=0, datatype=7, count=1),
            PointField(name="y", offset=4, datatype=7, count=1),
            PointField(name="z", offset=8, datatype=7, count=1),
            PointField(name="r", offset=12, datatype=7, count=1),
            PointField(name="g", offset=16, datatype=7, count=1),
            PointField(name="b", offset=20, datatype=7, count=1),
        ]
        return pc2.create_cloud(header, fields, points)

    def publish_fake_pointcloud(self):
        # read in image with opencv
        img = cv2.imread("fake_hose1_small.png")
        # print(img)
        pc_msg = Generator.image_to_pointcloud(img)
        self.cloud_pub.publish(pc_msg)


def main():
    rospy.init_node("kmeans_filter")
    r = rospy.Rate(10)
    filter = Generator()
    while not rospy.is_shutdown():
        filter.publish_fake_pointcloud()
        r.sleep()


if __name__ == "__main__":
    main()
