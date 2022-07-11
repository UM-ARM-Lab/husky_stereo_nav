#!/usr/bin/python3
import tf2_ros
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3


def main():
    pub = rospy.Publisher("loop_closure_count", Int32, queue_size=10)
    rospy.init_node("loop_closure_counter")
    rate = rospy.Rate(10.0)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    old_tf = TransformStamped()
    loop_closure_count = 0

    while not rospy.is_shutdown():
        try:
            tf = tf_buffer.lookup_transform(
                "odom", "map", rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF not found")
            rate.sleep()
            continue

        if not tf_equal(tf, old_tf):
            loop_closure_count += 1
            old_tf = tf
        
        pub.publish(loop_closure_count)
        rate.sleep()


def tf_equal(tf1, tf2):
    q1 = tf1.rotation
    q2 = tf2.rotation
    t1 = tf1.translation
    t2 = tf2.translation

    return (t1.x == t2.x and
            t1.y == t2.y and
            t1.z == t2.z and
            q1.x == q2.x and
            q1.y == q2.y and
            q1.z == q2.z and
            q1.w == q2.w)


if __name__ == "__main__":
    main()
