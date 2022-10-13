#!/usr/bin/python3
from matplotlib.colors import rgb_to_hsv
from color_utils import float_to_rgb
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
# import sklearn.cluster
# from colorsys import rgb_to_hsv
from matplotlib.colors import rgb_to_hsv
import matplotlib.pyplot as plt
import cv2


class Otsus:
    #cloud_pub: rospy.Publisher
    def __init__(self):
        rospy.Subscriber("/rtabmap/cloud_ground", PointCloud2, self.pc_callback)
        # rospy.Subscriber("/zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, self.pc_callback)
        self.cloud_pub1 = rospy.Publisher("/cloud_ground_hue", PointCloud2, queue_size=10)
        # self.cloud_pub2 = rospy.Publisher("/zed_hue", PointCloud2, queue_size=10)

    def pc_callback(self, msg: PointCloud2):
        # print(f"frame: {msg.header.frame_id}\n\n")
        # read point cloud into a Nx4 np array for [x, y, z, color]
        points = np.array(list(pc2.read_points(msg, skip_nans=True)))

        # convert intensity color vector to a Nx3 np array of [r, g, b], scaled as a float from [0, 1]
        float_rgb = np.vstack([float_to_rgb(intensity) for intensity in points[:, 3]]) / 255
        
        # convert rgb to Nx3 array of [h, s, v] all in range of [0, 1]
        hsv = rgb_to_hsv(float_rgb)

        # horizontally stack all column vectors of data we want to use for the overall cost metric
        z = points[:, 2]
        h = hsv[:, 0]
        
        # normalize z height to [0, 1]
        # z[z < -0.3] = -0.3
        # z -= np.min(z)
        # z /= np.max(z)
        print(float_rgb[:, 0].size)


        # cost_weights = np.array([1.0, 1.0, 1.0, 1.0])
        cost_weights = np.ones((2))
        cost_fields = np.column_stack((z, float_rgb[:, 0]))
        # cost_fields = float_rgb[:, 0]

        # compute the norm to get single value cost for each point
        costs = np.linalg.norm(cost_fields*cost_weights, axis=1)
        costs = float_rgb[:, 0]

        # print(z[z < -0.3].size)
        # print(z[z < 0].size)
        # print(z.size)
        # print(np.average(z))
        # plt.plot(z)
        # plt.show()
        # return
        
        # create an Nx3 array of [x, y, cost]
        cost_points = np.column_stack((points[:, :2], costs))
        # obstacle_ids = Otsus.otsus(cost_points)
        rgb_norm = np.linalg.norm(float_rgb, axis=1)

        # put all the new data columns on the end of the original point cloud matrix
        extra_data_points = np.column_stack((points, hsv, costs, rgb_norm, float_rgb))

        # add fields corresponding to the new data columns added
        fields = msg.fields
        fields.append(PointField(name="h", offset=20, datatype=7, count=1))
        fields.append(PointField(name="s", offset=24, datatype=7, count=1))
        fields.append(PointField(name="v", offset=28, datatype=7, count=1))
        fields.append(PointField(name="d", offset=32, datatype=7, count=1))
        fields.append(PointField(name="rgb_norm", offset=36, datatype=7, count=1))
        fields.append(PointField(name="r", offset=40, datatype=7, count=1))
        fields.append(PointField(name="g", offset=44, datatype=7, count=1))
        fields.append(PointField(name="b", offset=48, datatype=7, count=1))
        # print(f"fields: {fields}")
        # print(f"pointcloud is {msg.height} by {msg.width}")

        # convert point cloud matrix back to ROS point cloud type then publish it
        hue_cloud = pc2.create_cloud(msg.header, fields, extra_data_points)
        # if msg.header.frame_id == "zed2i_left_camera_frame":
        #     self.cloud_pub2.publish(hue_cloud)
        # else:
        # print(f"new pc is {hue_cloud.height} by {hue_cloud.width}")
        self.cloud_pub1.publish(hue_cloud)

    def otsus(cost_points: np.ndarray) -> np.ndarray:
        """
        :param cost_points: Nx3 array [[x, y, cost], ...]
                            cost = [0, 1]
        """
        
        # find min and max x and y
        x = cost_points[:, 0]
        y = cost_points[:, 1]
        min_x = np.min(x)
        max_x = np.max(x)
        min_y = np.min(y)
        max_y = np.max(y)
        x_size = max_x - min_x
        y_size = max_y - min_y
        pixel_size = 0.01
        
        # create an opencv matrix of size equal to some multiple of the pointcloud range
        # has an extra dimension for keeping track of averages
        x_pixels = np.ceil(x_size/pixel_size).astype(int)
        y_pixels = np.ceil(y_size/pixel_size).astype(int)
        mat = np.zeros((x_pixels, y_pixels, 2))
        
        # for each point in the cloud
            # divide its x and y by something to get its index in the image matrix
            # if the pixel at that spot is empty, assign the cost to it (mapped to 255)
            # if the pixel is not empty, average it ???

        for p in cost_points:
            x_id =  np.floor((p[0] - min_x)/pixel_size).astype(int)
            y_id =  np.floor((p[1] - min_y)/pixel_size).astype(int)

            mat[x_id, y_id, 0] = (mat[x_id, y_id, 0] + p[2]) / (mat[x_id, y_id, 1] + 1)
            mat[x_id, y_id, 1] += 1

        image = mat[:, :, 0] * 255
        cv2.imshow("img", image)
        cv2.waitKey(10)
        # run otsus method on the opencv matrix


def main():
    rospy.init_node("kmeans_filter")
    filter = Otsus()
    rospy.spin()

if __name__ == "__main__":
    main()