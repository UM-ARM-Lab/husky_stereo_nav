#!/usr/bin/python3
import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal


# every time a new goal is sent to move_base, 
# send a reset command to reset the trajectory server (on the /syscommand topic)
def main():
    global pub
    pub = rospy.Publisher("syscommand", String, queue_size=10)
    goal_sub = rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callback=nav_goal_callback)

    rospy.init_node("reset_path")
    rospy.spin()


def nav_goal_callback(data):
    global pub
    pub.publish("reset")


if __name__ == "__main__":
    main()
