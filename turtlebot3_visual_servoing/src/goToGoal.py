#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from geometry_msgs.msg import Pose,Quaternion


global GoalPose,CurrentPose
GoalPose = Pose()
CurrentPose = Pose()

def CallBack(msg):
        GoalPose.orientation.x = msg.orientation.x
        GoalPose.orientation.y = msg.orientation.y
        GoalPose.orientation.z = msg.orientation.z
        GoalPose.orientation.w = msg.orientation.w

        GoalPose.position.x = msg.position.x
        GoalPose.position.y = msg.position.y
        GoalPose.position.z = msg.position.z 

def CallBackcurrentPose(msg):
        CurrentPose.orientation.x = msg.orientation.x
        CurrentPose.orientation.y = msg.orientation.y
        CurrentPose.orientation.z = msg.orientation.z
        CurrentPose.orientation.w = msg.orientation.w

        CurrentPose.position.x = msg.position.x
        CurrentPose.position.y = msg.position.y
        CurrentPose.position.z = msg.position.z 

rospy.init_node("goToGoal")

sub = rospy.Subscriber("/GoalPose", Pose, CallBack)
sub = rospy.Subscriber("/CurrentPose", Pose, CallBackcurrentPose)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    inc_x = GoalPose.orientation.x  - CurrentPose.orientation.x
    inc_y = GoalPose.orientation.y - CurrentPose.orientation.y

    angle_to_goal = atan2(inc_y, inc_x)

    if abs(angle_to_goal - CurrentPose.orientation.w ) > 0.01:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    angle_to_orientent = atan2(GoalPose.orientation.w,CurrentPose.orientation.w)
    if abs(angle_to_orientent) > 0.01:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()    