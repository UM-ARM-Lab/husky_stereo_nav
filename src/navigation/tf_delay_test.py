#!/usr/bin/python3
import tf2_ros
import rospy
from sensor_msgs.msg import PointCloud2


def pointcloud_callback(msg):
    cloud_time = msg.header.stamp
    global tf_buffer
    try:
        tf = tf_buffer.lookup_transform(
            "odom", "base_link", cloud_time).transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.loginfo("TF not found")

    print(f"I found tf: {tf}")


# script for testing TF delay errors
def main():
    global tf_buffer
    pointcloud_sub = rospy.Subscriber(
        "zed2i/zed_node/point_cloud/cloud_registered", PointCloud2, callback=pointcloud_callback)
    rospy.init_node("test_tfs")
    rate = rospy.Rate(10.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.spin()
    # while not rospy.is_shutdown():

    #     try:
    #         tf = tf_buffer.lookup_transform(
    #             "odom", "base_link", rospy.Time()).transform
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         rospy.loginfo("TF not found")
    #         rate.sleep()
    #         continue

    #     print(f"I found tf: {tf}")
    #     rate.sleep()


if __name__ == "__main__":
    main()
