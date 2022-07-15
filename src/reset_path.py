#!/usr/bin/python3
import tf2_ros
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def main():
    global pub
    pub = rospy.Publisher("syscommand", String, queue_size=10)
    nav_goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, nav_goal_callback)
    rospy.init_node("reset_path")
    rospy.spin()

def nav_goal_callback(data):
    global pub
    pub.publish("reset")


if __name__ == "__main__":
    main()
