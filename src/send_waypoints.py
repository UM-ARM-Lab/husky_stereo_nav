#!/usr/bin/python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def main():
    rospy.init_node("waypoint_sender")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    while not client.wait_for_server(rospy.Duration(5.0)):
        rospy.INFO("waiting for move_base action server to start")
    
    goal = MoveBaseGoal
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.orientation.w = 1.0
    
    rospy.INFO("sending goal")
    client.send_goal(goal)
    
    client.wait_for_result()
    print(client.get_state())

if __name__ == "__main__":
    main()