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

Additionally, it's important to update the object detection model using TensorFlow Lite. 
In our case, we trained it with [Google Vertex AI](https://cloud.google.com/vertex-ai?hl=fr), a paid tool (offering 300 euros for each Google account, so you can train your model for free). Google Vertex AI provides an all-in-one solution with image labeling, cloud-based training with predefined presets. 
The model can also be created locally or using other tools, as long as it results in a TensorFlow Lite model. 
To update the model, you'll need to take multiple photos of the playing field with the different objects (or object states) placed on it. These images then need to be labeled, and the model trained with them.
Then you can export the model and replace the model.tflite file.


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
For the ESP WiFi, configure the Pi as an access point. For internet, use a second WiFi adapter or Ethernet.
Configuration example on Ubuntu:
Configure the hotspot
```
sudo nmcli con add type wifi ifname wlan0 con-name RobotConn autoconnect yes ssid Raspb
sudo nmcli con modify RobotConn 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared 802-11-wireless.channel 11
sudo nmcli con modify RobotConn wifi-sec.key-mgmt wpa-psk
sudo nmcli con modify RobotConn wifi-sec.psk "12345678»
```

And run it like this :
```
sudo nmcli con up RobotConn
```


## Components and Resources:

Here is the material the we used, but it should work with any wide lens camera. You don't need a such expensive camera. We use a broom handle to place the camera at 1 meter heigh (Balse.3fd) you will need to change the attach point to comply to the eurobot rules.

| Item                      | Description                           | Link                                                                                                             |
|---------------------------|---------------------------------------|------------------------------------------------------------------------------------------------------------------|
| ELP CMOS OV4689           | Camera Module                         | [Amazon FR](https://www.amazon.fr/gp/product/B07KMZ9GHZ/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)         |
| M12 × 0.5 185° Lens       | Wide-Angle Lens for Camera            | [Amazon FR](https://www.amazon.fr/gp/product/B07MFYM133/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)         |
| Brostrend 1200Mbps Wi-Fi Dongle | Optional Wi-Fi Dongle for Raspberry Pi | [Brostrend](https://www.brostrend.com/collections/france/products/brostrend-1200mbps-linux-usb-cle-wifi-adaptateurs-longue-portee) |
| Raspberry Pi 4 (4GB)      | Main Microcontroller                  | [Official Site](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/)                                   |


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
     
5. **TagsAngle Node**: 
   - Subscribes to: `/cam1` and not to `/terrain` if the tag is outside of the terrain (solar pannel).
   - Function: Identifies and locates tags in the image.
   - Publishes: `/tagsAngle` (Float32MultiArray) indicating the position and identification and orientation of the tag. (folling format [id1,angle1,id2,angle1,x2,y2.....idn,anglen])

6. **Astar Node**: 
   - Subscribes to: `/mask` for obstacle data.
   - Function: Calculates the optimal path avoiding obstacles.
   - Publishes: `/path` which outlines the computed path.

7. **RobotCommunicationWifi Node**: 
   - Function: Send and recive an example playload via wifi between the Pi and the ESP32

8. **RobotCommunicationBLE Node**: 
   - Function: Send and recive an example playload via bluetooth between the Pi and the ESP32

9. **ObjectDetection Node**: 
   - Subscribes to: `/terrain`
   - Function: Using tensorflow (MobileNet) it detect the different type of object and the position. You can modify the treshold of the score in the .py file
   - Publishes: `/obj` (Float32MultiArray) indicating the position and identification of the object. (folling format [id1,x1,y1,id2,x2,y2.....idn,xn,yn])

10. Grid
  - Subscribe to `/mask`
  - Publishes: `/map` an occupency grid (usefull if you are using to RVIZ to display the map)

11. ImageTaker
  - Subscribe to `/terrain`
  - Run it to create a referance image of the empty terrain

To use those information, create a Node that subscribe to one a multiple topics : 
Here is a tutorial : http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

