# ECAM Eurobot 2024 - Team Erasmus

Compatible with Raspberry Pi 4. For better performance, use a laptop with Ubuntu (dual boot or VM). A laptop can fit in the space of 1600x450x320mm³.

## Prerequisites
* Ubuntu 20.04, might work on 18.04. 
* ROS Noetic
* opencv-python >= 4.8
* cv_bridge
* tflite
* bluez, python-bluez

## Bluetooth Setup Instructions
- Install bluez: `sudo apt-get install bluetooth bluez`
- Install pyBluez: `sudo apt-get install bluez python3-bluez`
- To scan BLE devices: 
  ```
  python
  import bluetooth
  print(bluetooth.discover_devices(lookup_names=True))
  ```


### ROS Installation
Package designed for ROS1 Noetic (not tested on other versions).
Follow the ROS Noetic installation guide: [ROS Noetic Installation for Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)  
### Workspace Configuration
Initialize the workspace:
```
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
More information on : http://wiki.ros.org/catkin/Tutorials/create_a_workspace

### Package Installation
Clone the project repository and build the package:
```
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/TardyNoe/Balise.git
cd ~/catkin_ws/
catkin build
```
### Camera Calibration
Calibrate your camera : You can use the ros package : [Monocular Camera Calibration ROS Package](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).  
Or use python and cv2 : [Monocular Camera Calibration Python and cv2]https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
Update the camera resolution and matrix (mtx and dist) in the Balise/src/CameraPublisher.py file due to lens distortion.

### Configuration
The program is pre-configured for the 2024 Eurobot challenge.
If Aruco tag positions change in future challenges, update the image in Balise/src/table.png.

### Launch Instructions
Launch the core in one terminal:
```
source ~/catkin_ws/devel/setup.bash
roslaunch Balise Full.launch
```

### Additional Information for Raspberry Pi Users
If using a Raspberry Pi, you can preview the video feed over a local network (note: this may slow down the Pi and preview a slow frame rate). 
Refer to ROS Network Setup : http://wiki.ros.org/ROS/NetworkSetup. You also need to install ROS on the machine that preview the video feed (use a Ubutnu VM and rviz)

## Robot connexion :
For WiFi, configure the Pi as an access point. For internet, use a second WiFi adapter or Ethernet.
Configuration example on Ubuntu:
You can do like this on ubutnu :
sudo nmcli con add type wifi ifname wlan0 con-name RobotConn autoconnect yes ssid Raspb
sudo nmcli con modify RobotConn 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared 802-11-wireless.channel 11
sudo nmcli con modify RobotConn wifi-sec.key-mgmt wpa-psk
sudo nmcli con modify RobotConn wifi-sec.psk "12345678»

And run it like this :
sudo nmcli con up RobotConn

## System Architecture
To run a single node use : rosrun <nodename>
The roslaunch command let you run multiple node using a launch file

1. **Camera Node**:
   - Function: open the camera on the device
   - Publishes: `/cam1`, which contains the undistorded video feed of the camera.

2. **Terrain Node**: 
   - Subscribes to: `/cam1` for receiving camera inputs.
   - Publishes: `/terrain`, which contains the straightened terrain image.

3. **Mask Node**: 
   - Subscribes to: `/terrain` for the straightened terrain image.
   - Function: Compares the received image to a reference image.
   - Publishes: `/mask`, which is the mask of obstacles on the terrain (binary image).

4. **TagsPosition Node**: 
   - Subscribes to: `/terrain` for the straightened terrain image.
   - Function: Identifies and locates tags in the image.
   - Publishes: `/tags` (Float32MultiArray) indicating the position and identification and orientation of the tag. (folling format [id1,angle1,x1,y1,id2,angle1,x2,y2.....idn,anglen,xn,yn])

5. **Astar Node**: 
   - Subscribes to: `/mask` for obstacle data.
   - Function: Calculates the optimal path avoiding obstacles.
   - Publishes: `/path` which outlines the computed path.

6. **RobotCommunicationWifi Node**: 
   - Function: Send and recive an example playload via wifi between the Pi and the ESP32

7. **RobotCommunicationBLE Node**: 
   - Function: Send and recive an example playload via bluetooth between the Pi and the ESP32

8. **ObjectDetection Node**: 
   - Subscribes to: `/terrain`
   - Function: Using tensorflow (MobileNet) it detect the different type of object and the position. You can modify the treshold of the score in the .py file
   - Publishes: `/obj` (Float32MultiArray) indicating the position and identification of the object. (folling format [id1,x1,y1,id2,x2,y2.....idn,xn,yn])

9. Grid
  - Subscribe to `/mask`
  - Publishes: `/map` an occupency grid (usefull if you are using to RVIZ to display the map)

10. ImageTaker
  - Subscribe to `/terrain`
  - Run it to create a referance image of the empty terrain

To use those information, create a Node that subscribe to one a multiple topics : 
Here is a tutorial : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

