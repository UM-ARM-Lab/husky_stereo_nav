#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# count and print the number of loop closures reported by RTAB-Map
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
