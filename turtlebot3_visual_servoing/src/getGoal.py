#!/usr/bin/env python

#import
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import yaml
import numpy as np
import rospkg
cv_object = CvBridge()


#get package path
rospack = rospkg.RosPack()
packagePath = rospack.get_path('turtlebot3_visual_servoing')

# HSV filtering values for the obstracle
H_Min = 0
S_Min = 138
V_Min = 255

H_Max = 17
S_Max = 255
V_Max = 255




def Callback(msg):
    # convert image msg to image 
    image = cv_object.imgmsg_to_cv2(msg,'bgr8')
    # get aurco dictionary and params
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create()
    # detect the aurco markers and its corner
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
	                parameters=arucoParams) 
    if  len(corners) > 0:
        index = ids.flatten()
        # this aurco marker id for detection.
        if  index[0] == 7:
            print(index )
            # save the goal pose
            print('*****saving the goal Pose******')
            cv2.imwrite(packagePath+'/navigation/goalPose.png',image)
            # filter the obstracle
            hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)   
            mask = cv2.inRange(hsv,tuple([H_Min,S_Min,V_Min]),tuple([H_Max,S_Max,V_Max]))
            kernel = np.ones((5, 5), np.uint8)
            # create the map             
            mask = cv2.erode(mask, kernel,iterations=1)
            mask = cv2.dilate(mask,kernel,iterations=5)
            # save the map for navigation
            mask = cv2.bitwise_not(mask) 
            print('*****saving the Map ******')       
            cv2.imwrite(packagePath+'/navigation/map.pgm',mask)




def main():
    # subscribe to the topic
    sub  = rospy.Subscriber('/camera/image_raw',Image,Callback)
    while not rospy.is_shutdown():
        rospy.spin()


# initlize the ros node
if __name__ == '__main__':
    rospy.init_node('getGoal')
    try :
        main()
    except rospy.ROSException() as e :
        print(e)