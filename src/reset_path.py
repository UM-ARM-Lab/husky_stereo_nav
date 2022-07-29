#!/usr/bin/python3
import tf2_ros
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction


def main():
    global pub
    pub = rospy.Publisher("syscommand", String, queue_size=10)
    nav_goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped, nav_goal_callback)

    client = SimpleActionClient("move_base", MoveBaseAction, execute_cb=nav_goal_callback)
    rospy.init_node("reset_path")
    rospy.spin()

def nav_goal_callback(data):
    global pub
    pub.publish("reset")


if __name__ == "__main__":
    main()
