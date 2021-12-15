#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from geometry_msgs.msg import Pose,Quaternion

global GoalPose,goal
GoalPose = Pose()
goal = MoveBaseGoal()
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)


def callback(msg):
        GoalPose.orientation.x = msg.orientation.x
        GoalPose.orientation.y = msg.orientation.y
        GoalPose.orientation.z = msg.orientation.z
        GoalPose.orientation.w = msg.orientation.w

        GoalPose.position.x = msg.position.x
        GoalPose.position.y = msg.position.y
        GoalPose.position.z = msg.position.z 
        try :
            goal.target_pose.pose.position.x = GoalPose.position.x
            goal.target_pose.pose.position.y = GoalPose.position.y
            goal.target_pose.pose.position.z = GoalPose.position.z
            goal.target_pose.pose.orientation.x = GoalPose.orientation.x
            goal.target_pose.pose.orientation.y = GoalPose.orientation.y
            goal.target_pose.pose.orientation.z = GoalPose.orientation.z
            goal.target_pose.pose.orientation.w = GoalPose.orientation.w
            client.send_goal(goal)
        except rospy.ROSException as e:
            print(e)

def main():
    client.wait_for_server()




if __name__ == '__main__':
    rospy.init_node('navigateObstacle')
    sub = rospy.Subscriber('/GoalPose',Pose,callback)
    try: 
        main()
    except rospy.ROSException as e:
        print(e)