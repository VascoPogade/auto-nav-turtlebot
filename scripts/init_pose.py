#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import threading


class Velocity:
    def __init__(self, angular, linear):
        self.angular = angular
        self.linear = linear


class Linear:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class Angular:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


def update_twist(velocity, publisher):
    twist = Twist()

    twist.linear.x = velocity.linear.x
    twist.linear.y = velocity.linear.y
    twist.linear.z = velocity.linear.z

    twist.angular.x = velocity.angular.x
    twist.angular.y = velocity.angular.y
    twist.angular.z = velocity.angular.z

    publisher.publish(twist)
    print("Velocity changed!")

def are_odoms_equal(odom1, odom2, tolerance=0.2):
    """
    Check if two odom objects are equal within a specified tolerance.

    Parameters:
    - odom1: The first odom object.
    - odom2: The second odom object.
    - tolerance: The allowed difference in coordinates.

    Returns:
    - True if the odom objects are equal within the specified tolerance, False otherwise.
    """

    # Check if the coordinates are within the allowed tolerance
    x_diff = abs(odom1.pose.pose.orientation.x - odom2.pose.pose.orientation.x)
    y_diff = abs(odom1.pose.pose.orientation.y - odom2.pose.pose.orientation.y)
    z_diff = abs(odom1.pose.pose.orientation.z - odom2.pose.pose.orientation.z)

    return x_diff <= tolerance and y_diff <= tolerance and z_diff <= tolerance


# Node initialization
rospy.init_node('init_pose')
initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"

init_msg.pose.pose.position.x = 1.95
init_msg.pose.pose.position.y = 0.43
init_msg.pose.pose.orientation.x = 0.0
init_msg.pose.pose.orientation.y = 0.0
init_msg.pose.pose.orientation.z = 0.39
init_msg.pose.pose.orientation.w = 0.91
init_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

# Delay
rospy.sleep(1)

# Publish message
rospy.loginfo("setting initial pose")
initialpose_pub.publish(init_msg)
rospy.loginfo("initial pose is set")

current_loc = rospy.wait_for_message('/odom', Odometry)
rospy.sleep(1)

turned = False

def turn():
    while not turned:
        rospy.sleep(1)
        ang = Angular(z=0.5)
        lin = Linear()
        vel = Velocity(angular=ang, linear=lin)
        update_twist(vel, cmd_vel_pub)
    print("Initial turn done!")

my_thread = threading.Thread(target=turn)

my_thread.start()

rospy.sleep(3)
while not turned:
    odom2 = rospy.wait_for_message('/odom', Odometry)
    if are_odoms_equal(current_loc, odom2, tolerance=0.02):
        turned = True

print("Inital pose done!")
