#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

rospy.init_node('TagDetectorAngle')


bridge = CvBridge()


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    corners, ids, _ = detector.detectMarkers(cv_image)
    
    data = []
    
    if ids is not None:
        for i in range(len(corners)):
            for j in range(len(corners[i])):
                p = corners[i][j]
                angle = np.degrees(np.arctan2(p[1][0]-p[0][0],p[1][1]-p[0][1]))
                idtag = ids[i][0]
                data.append(idtag)
                data.append(angle)
    
    Tags_msg = Float32MultiArray(data=np.array(data))
    pubTags.publish(Tags_msg)

pubTags = rospy.Publisher('/TagsAngle', Float32MultiArray, queue_size=10)
image_sub = rospy.Subscriber('/cam1', Image, image_callback)
rospy.spin()

