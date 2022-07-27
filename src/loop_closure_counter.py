#!/usr/bin/python3
import tf2_ros
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    sub = rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, loop_closure_callback)
    rospy.init_node("loop_closure_counter")

    global loop_closure_count
    loop_closure_count = 0

    rospy.spin()

def loop_closure_callback(msg):
    global loop_closure_count
    loop_closure_count += 1
    rospy.loginfo(f"Loop Closure detected, count = {loop_closure_count}")

if __name__ == "__main__":
    main()
