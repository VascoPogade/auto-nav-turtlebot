#!/usr/bin/env python3

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray


# Callbacks definition
def active_cb(extra):
    rospy.loginfo("Goal pose being processed")


def feedback_cb(feedback):
    rospy.loginfo("Current location: " + str(feedback))


def done_cb(status, result):
    if status == 3:
        rospy.loginfo("Goal reached")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")


# Function to navigate to a specific location
def move_to_goal(pos_x, pos_y, ori_z, ori_w):
    nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    nav_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = pos_x
    goal.target_pose.pose.position.y = pos_y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = ori_z
    goal.target_pose.pose.orientation.w = ori_w

    nav_client.send_goal(goal, done_cb, active_cb, feedback_cb)
    finished = nav_client.wait_for_result()

    if not finished:
        rospy.logerr("Action server not available!")
    else:
        rospy.loginfo(nav_client.get_result())


# Check if args were entered
if len(sys.argv) != 5:
    print("Usage: rosrun auto_nav goal_pose.py <pos_x> <pos_y> <ori_z> <ori_w>")
    sys.exit(1)
else:
    rospy.init_node('goal_pose')
    pos_x, pos_y, ori_z, ori_w = map(float, sys.argv[1:5])
    print("waiting for status!")
    status_msg = rospy.wait_for_message('/move_base/status', GoalStatusArray)
    print("status received!")
    try:
        status = status_msg.status_list[-1].status
    except IndexError:
        status = 41

    if status in [2, 3, 4, 5, 8, 41]:
        move_to_goal(pos_x, pos_y, ori_z, ori_w)
    else:
        rospy.loginfo("Turtlebot is currently busy!")
