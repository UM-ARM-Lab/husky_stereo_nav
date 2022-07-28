#!/usr/bin/python3
import json
import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

# send the given waypoint to the move_base action server as a goal


def send_waypoint(client: SimpleActionClient, goal_pose: Pose, wait_between_goals: bool):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = goal_pose

    rospy.loginfo("sending goal")
    client.send_goal(goal)
    client.wait_for_result()

    rospy.loginfo(f"goal ended with state: {client.get_state()}")
    if wait_between_goals:
        input("press enter to continue to next waypoint")


def state_num_to_str(state_num: int) -> str:
    if state_num == 0:
        return "PENDING"
    elif state_num == 1:
        return "ACTIVE"
    elif state_num == 2:
        return "PREEMPTED"
    elif state_num == 3:
        return "SUCCEEDED"
    elif state_num == 4:
        return "ABORTED"
    elif state_num == 5:
        return "REJECTED"
    elif state_num == 6:
        return "PREEMPTING"
    elif state_num == 7:
        return "RECALLING"
    elif state_num == 8:
        return "RECALLED"
    elif state_num == 9:
        return "LOST"
    else:
        return "INVALID STATE"


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

    # get wait parameter, default to true
    wait_between_goals = rospy.get_param("wait_between_goals", True)

    # wait for action client to start
    while not client.wait_for_server(rospy.Duration(5.0)):
        rospy.loginfo("waiting for move_base action server to start")

    # read waypoints from a JSON file, given as a ROS parameter
    try:
        json_file_path = rospy.get_param("waypoint_sequence_path")
    except KeyError:
        rospy.logerr(
            "No JSON file provided, parameter waypoint_sequence_path needs to be set")

    with open(json_file_path, "r") as f:
        waypoint_dict = json.load(f)
        waypoints = [dict_to_pose(w) for w in waypoint_dict]

    # send waypoints sequentially to action server
    for w in waypoints:
        send_waypoint(client, w, wait_between_goals)

    rospy.loginfo("All waypoints complete!")


if __name__ == "__main__":
    main()
