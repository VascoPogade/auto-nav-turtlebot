#!/usr/bin/env python3

import rospy
import actionlib
import sys
import argparse
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


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Navigate robot to certain goal. Please supply the goal pose as the name of the pickup point')
    parser.add_argument('pickup_point', type=str, help='Name of the pickup point')

    args = parser.parse_args()
    pickup_point = args.pickup_point

    rospy.init_node('goal_pose')
    #there may be problems with the missing line of mapping
    print("waiting for status!")
    status_msg = rospy.wait_for_message('/move_base/status', GoalStatusArray)
    print("status received!")

    try:
        status = status_msg.status_list[-1].status
    except IndexError:
        status = 41

    if status in [2, 3, 4, 5, 8, 41]:
        if pickup_point == "pickup_point1":
            move_to_goal(11.612991333007812,1.8986226320266724 , -0.6767184325390025, 0.7362419188432261) #example coordinates
        elif pickup_point == "pickup_point2":
            move_to_goal(-0.38340112566947937,-0.08671373128890991, 0.024248888396507462,  0.999705952473793) #example coordinates
        elif pickup_point == "pickup_point3":
            move_to_goal(14.80724048614502,0.6164495944976807 ,0.026950692485214478, 0.9996367641171304) #example coordinates
        elif pickup_point == "pickup_point4":
            move_to_goal(22.618375778198242,0.4330832362174988,-0.9998970198030099 ,0.014350950806798986) #example coordinates
        elif pickup_point =="pickup_point5":
            move_to_goal(7.000452995300293, 1.772918462753296, -0.6893069327700934, 0.7244694282266754)
        elif pickup_point =="pickup_point6":
            move_to_goal(20.220516204833984, 1.0709266662597656, -0.999519797879735, 0.030986668850551045)
        else:
            rospy.loginfo("Invalid pickup point!")
    else:
        rospy.loginfo("Turtlebot is currently busy!")