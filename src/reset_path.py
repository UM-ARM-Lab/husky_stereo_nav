#!/usr/bin/python3
import tf2_ros
import rospy
from actionlib import SimpleActionClient
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal


def main():
    global pub
    pub = rospy.Publisher("syscommand", String, queue_size=10)
    goal_sub = rospy.Subscriber(
        "move_base/goal", MoveBaseActionGoal, callback=nav_goal_callback)

    rospy.init_node("reset_path")
    rospy.spin()


def nav_goal_callback(data):
    global pub
    pub.publish("reset")


if __name__ == "__main__":
    main()
