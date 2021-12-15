#!/usr/bin/env python

# import
import rospy
from rospy.exceptions import TransportException
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import yaml
import numpy as np
import rospkg
from geometry_msgs.msg import Pose,Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

#get package path
rospack = rospkg.RosPack()
packagePath = rospack.get_path('turtlebot3_visual_servoing')


class DetectAurco:
    def __init__(self):
        with open(packagePath+"/calibrationdata/ost.yaml", "r") as stream:
            try:
                calibrationData = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        
        self.GoalPose = Pose()
        self.CurrentPose = Pose()
        self.Odom = Odometry()
        self.cameraMatrix = np.array(calibrationData['camera_matrix']['data']).reshape(3,3)
        self.distortionMatrix = np.array(calibrationData['distortion_coefficients']['data'])
        self.cv_object = CvBridge()
        self.Image = None
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.currentPose = None
        self.TransformFromCurrentToGoal  = None

        self.sub = rospy.Subscriber('/camera/image_raw',Image,self.Callback)
        self.pub = rospy.Publisher('/currentPoseImage',Image,queue_size=1)
        self.GoalPosePub = rospy.Publisher('/GoalPose',Pose,queue_size=10)
        self.CurrentPosePub = rospy.Publisher('/CurrentPose',Pose,queue_size=10)
        self.odometry = rospy.Publisher('/odm',Odometry,queue_size=10)

        self.goalPose = self.getgoalPose()

    def getgoalPose(self):
        image = cv2.imread(packagePath+'/navigation/goalPose.png')
        (corners, ids, rejected) = self.getcorners(image)
        rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners, 0.2 , self.cameraMatrix,
                                                                            self.distortionMatrix)
        rotation_matrix = np.zeros(shape=(3,3))
        cv2.Rodrigues(rvec,rotation_matrix)
        tvec = tvec.flatten()
        TransformationMatrix = np.vstack( [np.hstack([rotation_matrix,tvec.reshape(-1,1)])  , [0,0,0,1]])
        print("Transformation from goal  to camera :")
        print(TransformationMatrix)
        rvec = rvec.flatten()
        quaternion = quaternion_from_euler(rvec[0],rvec[1],rvec[2])
        # print(quaternion)
        self.GoalPose.orientation.x = quaternion[0]
        self.GoalPose.orientation.y = quaternion[1]
        self.GoalPose.orientation.z = quaternion[2]
        self.GoalPose.orientation.w = quaternion[3]

        self.GoalPose.position.x = tvec[0]
        self.GoalPose.position.y = tvec[1]
        self.GoalPose.position.z = tvec[2]

        self.GoalPosePub.publish(self.GoalPose)


        return TransformationMatrix



     
    def Callback(self,msg):
        self.GoalPosePub.publish(self.GoalPose)
        image = self.cv_object.imgmsg_to_cv2(msg,'bgr8')
        self.Image = image
        if self.goalPose.any() == None:
            self.getgoalPose()
        print(self.goalPose)
        # print(image.shape)

        (corners, ids, rejected) = self.getcorners(self.Image)
        if len(corners) > 0:
            index = ids.flatten()
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.2 , self.cameraMatrix,self.distortionMatrix)
                rotation_matrix = np.zeros(shape=(3,3))
                cv2.Rodrigues(rvec,rotation_matrix)
                tvec = tvec.flatten()

                TransformationMatrix = np.vstack( [np.hstack([rotation_matrix,tvec.reshape(-1,1)])  , [0,0,0,1]])

                rvec = rvec.flatten()


                # Draw a square around the markers
                corner = corners[i].reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corner
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                if index[i] == 7:

                    quaternion = quaternion_from_euler(rvec[0],rvec[1],rvec[2])
                    # print(quaternion)
                    self.CurrentPose.orientation.x = quaternion[0]
                    self.CurrentPose.orientation.y = quaternion[1]
                    self.CurrentPose.orientation.z = quaternion[2]
                    self.CurrentPose.orientation.w = quaternion[3]

                    self.CurrentPose.position.x = tvec[0]
                    self.CurrentPose.position.y = tvec[1]
                    self.CurrentPose.position.z = tvec[2]

                    self.Odom.pose.pose = self.CurrentPose

                    self.CurrentPosePub.publish(self.CurrentPose)
                    self.odometry.publish(self.Odom)

                    color = (0,255,0)
                    self.goalPose = TransformationMatrix
                    print("Transformation from goal  to camera :")
                    print(self.goalPose)



                    cv2.line(image, topLeft, topRight, color, 2)
                    cv2.line(image, topRight, bottomRight, color, 2)
                    cv2.line(image, bottomRight, bottomLeft, color, 2)
                    cv2.line(image, bottomLeft, topLeft, color, 2)
                    # Draw Axis
                    cv2.aruco.drawAxis(image, self.cameraMatrix, self.distortionMatrix, rvec, tvec, 0.01) 

        if self.goalPose is not None and self.currentPose is not None:
            inverse = np.linalg.inv(self.goalPose)
            self.TransformFromCurrentToGoal = np.matmul(self.currentPose,inverse)
            print("*********************")
            print("Transformation from current to Gaol :")
            print(self.TransformFromCurrentToGoal)
            # print(self.currentPose.shape)
            # print(np.linalg.inv(self.goalPose))
        
        self.pub.publish(self.cv_object.cv2_to_imgmsg(image,"bgr8"))

 

    def getcorners(self,image):
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
	                parameters=arucoParams)        
        return (corners, ids, rejected)
        
        
        


if __name__ == '__main__':
    rospy.init_node('currentPose')
    try :
        L = DetectAurco()
        rospy.sleep(200)
        rospy.spin()
  
    except rospy.ROSException as e:
        print(e)