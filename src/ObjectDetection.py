#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge  
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import tflite_runtime.interpreter as tflite
import time

# Load TensorFlow Lite model
tflite_model_path = '/home/balise/catkin_ws/src/Balise/src/model.tflite' 
interpreter = tflite.Interpreter(model_path=tflite_model_path)
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Create an instance of CvBridge
bridge = CvBridge()

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # Convert ROS Image message to OpenCV image
    except Exception as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    img = cv2.resize(cv_image, (512, 512), interpolation=cv2.INTER_AREA)
    img_array = [img]
    interpreter.set_tensor(input_details[0]['index'], img_array)
    interpreter.invoke()
    boxes = interpreter.get_tensor(output_details[0]['index'])
    classes = interpreter.get_tensor(output_details[1]['index'])
    scores = interpreter.get_tensor(output_details[2]['index'])
    threshold = 0.2

    plantsUp = []
    plantsDown = []
    robot = []

    image = img_array[0]
    for box, cls, score in zip(boxes[0], classes[0], scores[0]):
        if score > threshold:
            y1, x1, y2, x2 = box
            color = (50*cls, 50*cls, 255-50*cls)
            centerx = int((x1 + x2) * 512 / 2)
            centery = int((y1 + y2) * 512 / 2)
            image = cv2.circle(image, (centerx, centery), 8, color, 3)
            posx = (-(centerx - 400) / 200)
            posy = (-(centery - 300) / 200)
            if cls == 2:
                plantsUp.append(posx)
                plantsUp.append(posy)
            elif cls == 1:
                plantsDown.append(posx)
                plantsDown.append(posy)
            elif cls == 0:
                robot.append(posx)
                robot.append(posy)

    image = cv2.resize(image, (400, 600), interpolation=cv2.INTER_AREA)
    image_message = bridge.cv2_to_imgmsg(image, "bgr8")
    pub.publish(image_message)

    plantsUp_msg = Float32MultiArray(data=np.array(plantsUp))
    plantsDown_msg = Float32MultiArray(data=np.array(plantsDown))
    robot_msg = Float32MultiArray(data=np.array(robot))

    plantsUp_pub.publish(plantsUp_msg)
    plantsDown_pub.publish(plantsDown_msg)
    robot_pub.publish(robot_msg)

if __name__ == '__main__':
    rospy.init_node('ObjectDetection', anonymous=True)
    
    rospy.Subscriber('/terrain', Image, image_callback)
    
    plantsUp_pub = rospy.Publisher('/obj/plantsUp', Float32MultiArray, queue_size=10)
    plantsDown_pub = rospy.Publisher('/obj/plantsDown', Float32MultiArray, queue_size=10)
    robot_pub = rospy.Publisher('/obj/robot', Float32MultiArray, queue_size=10)
    
    pub = rospy.Publisher('/Detection', Image, queue_size=10)
    
    rospy.spin()

