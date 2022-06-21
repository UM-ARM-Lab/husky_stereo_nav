#!/usr/bin/python3
import json
import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

# send the given waypoint to the move_base action server as a goal
def send_waypoint(client: SimpleActionClient, goal_pose: Pose):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = goal_pose

    rospy.loginfo("sending goal")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(f"goal ended with state: {client.get_state()}")
    input("press enter to continue to next waypoint")


# convert a json like dictionary containing pose data into a ROS Pose message
def dict_to_pose(pose_dict: dict) -> Pose:
    pose = Pose()
    pose.position = Point(x=pose_dict["position"]["x"],
                          y=pose_dict["position"]["y"],
                          z=pose_dict["position"]["z"])
    pose.orientation = Quaternion(x=pose_dict["orientation"]["x"],
                                  y=pose_dict["orientation"]["y"],
                                  z=pose_dict["orientation"]["z"],
                                  w=pose_dict["orientation"]["w"])
    return pose


def main():

    # initialize node and action client
    rospy.init_node("waypoint_sender")
    client = SimpleActionClient("move_base", MoveBaseAction)

    # wait for action client to start
    while not client.wait_for_server(rospy.Duration(5.0)):
        rospy.loginfo("waiting for move_base action server to start")

    # read waypoints from json file
    # TODO: dont hardcode path
    with open("/home/armlab/catkin_ws/src/husky-stereo-nav/src/lab-waypoints.json", "r") as f:
        waypoint_dict = json.load(f)
        waypoints = [dict_to_pose(w) for w in waypoint_dict]

    # send waypoints sequentially to action server
    for w in waypoints:
        send_waypoint(client, w)



if __name__ == "__main__":
    main()
