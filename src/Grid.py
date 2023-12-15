#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

def cam_callback(image_message):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to OpenCV format using cv_bridge
        cv_image = bridge.imgmsg_to_cv2(image_message, "mono8")
        cv_image = cv2.resize(cv_image, (40, 60), interpolation=cv2.INTER_AREA)

        # Process the image to create an OccupancyGrid
        grid = create_occupancy_grid(cv_image)

        # Publish the OccupancyGrid
        pub.publish(grid)

    except CvBridgeError as e:
        rospy.logerr("Error in cam_callback: %s", e)

def create_occupancy_grid(binary_image):
    grid = OccupancyGrid()
    grid.header = Header()
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = "map"

    # Set grid info
    resolution = 3.0 / 60  # meters per pixel
    grid.info.resolution = resolution
    grid.info.width = binary_image.shape[1]
    grid.info.height = binary_image.shape[0]

    # Set the origin (adjust as needed)
    grid.info.origin.position.x = (grid.info.width - 40) * resolution
    grid.info.origin.position.y = -30 * resolution
    grid.info.origin.position.z = 0.0

    # Populate the grid data
    for i in range(grid.info.height):
        for j in range(grid.info.width - 1, -1, -1):  # Invert x-axis
            pixel = binary_image[i, j]
            if pixel == 255:  # Assuming white pixels are free space
                grid.data.append(0)
            else:
                grid.data.append(100)  # Occupied space

    return grid

if __name__ == '__main__':
    rospy.init_node('occupancy_grid_publisher')

    # Create a publisher
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

    # Create a subscriber
    rospy.Subscriber("mask", Image, cam_callback)

    rospy.spin()

