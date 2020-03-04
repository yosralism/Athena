# Athena

Athena is a hexacopter made for National Flying Robot Competition 2019 with a mission to drop 7 payloads at certain locations autonomously with the help of computer vision.   

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install on the vehicle's onboard PC

* Ubuntu 18.04
* [OpenCV 3](https://docs.opencv.org/3.4/d2/de6/tutorial_py_setup_in_ubuntu.html) - Library for computer vision
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - Framework used
* [Mavros](https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html) - The mavros ROS package enables MAVLink extendable     communication
* [Imutils Package](https://pypi.org/project/imutils/) - Enhance CV performance 
* [ROS PID Package](http://wiki.ros.org/pid) - Plug and Play PID Package for ROS
* [Arduino IDE](https://ubuntu.com/tutorials/install-the-arduino-ide#1-overview) - Used to program payloads dropping mechanism
* [Rosserial Arduino](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) Package for communication between ROS and Arduino

### How To Build

Go to your catkin_workspace/your_src folder
```
git clone https://github.com/yosralism/athena.git
cd athena
source devel/setup.bash
catkin build
```
## Running the tests 

Connect your ground station (laptop) to the vehicle's onboard PC with WiFi router via ssh
```
ssh device_name@ip_address
```
example
```
ssh odroid@192.168.1.105
```
### Checking up servo mechanism
```
roslaunch athena servo_test.launch
```
### Run the full mission test
```
roslaunch mavros apm.launch
```
Open new terminal

```
roslaunch athena athena_guided.launch
```

Important to check:
* Flight mode 
* Waypoint reach

For checking target detection value, open new terminal
```
rostopic echo /cv_target
```

