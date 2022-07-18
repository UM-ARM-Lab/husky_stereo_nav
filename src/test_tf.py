#!/usr/bin/python3
import tf2_ros
import rospy


def main():
    rospy.init_node("test_tfs")
    rate = rospy.Rate(10.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    while not rospy.is_shutdown():
        try:
            tf = tf_buffer.lookup_transform(
                "base_footprint", "zed2i_left_camera_optical_frame", rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF not found")
            rate.sleep()
            continue

        print(f"I found tf: {tf}")
        rate.sleep()


if __name__ == "__main__":
    main()
