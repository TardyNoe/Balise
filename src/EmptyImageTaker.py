#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

EmptyImagePath = '/home/balise/catkin_ws/src/Balise/src/EmptyImage.png'

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV image
        cv_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    # Save the image
    cv2.imwrite(EmptyImagePath, cv_img)

    # Shutdown the node
    rospy.signal_shutdown("Image captured and saved.")

def main():
    rospy.init_node('image_listener', anonymous=True)
    
    # Subscribe to the /terrain topic
    rospy.Subscriber("/terrain", Image, image_callback)

    # Keep python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()

