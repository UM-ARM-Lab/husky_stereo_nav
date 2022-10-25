#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from color_utils import float_to_rgb
import sensor_msgs.point_cloud2 as pc2

def pc_callback(msg: PointCloud2):
    points = np.array(list(pc2.read_points(msg, skip_nans=True)))
    if points.shape[1] == 4:
        rgb_points = np.vstack([float_to_rgb(intensity) for intensity in points[:, 3]]) / 255
        points = np.column_stack((points[:, :3], rgb_points))

    np.savetxt(f"pc_frame_fake{msg.header.stamp}.txt", points)

def main():
    rospy.init_node("pointcloud_frame_saver")
    # rospy.Subscriber("zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, pc_callback)
    # rospy.spin()
    # msg = rospy.wait_for_message("zed2i/zed_node/point_cloud/cloud_registered", PointCloud2)
    msg = rospy.wait_for_message("fake_hose_cloud", PointCloud2)
    pc_callback(msg)
    
    
if __name__ == "__main__":
    main()