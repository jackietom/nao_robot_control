# nao_robot_control
projects to control a nao robot through kinect

This project is divided in two parts: kinect part and nao robot part

##### Installation

Environment: ubuntu:14.04 ROS:indigo

1. Kinect

```
cd nao_robot_control/kinect/NITE-Bin-v-Linux-v1.5.2.23/x64/
sudo ./install.sh
cd nao_robot_control/kinect/SensorKinect/Bin/Sensor-Bin-Linux-x64-v5.1.2.1
sudo ./install.sh

cd nao_robot_control/kinect/openTracker_ws
rm -r build devel
catkin_make
```

2. Nao robot

```
sudo apt-get install ros-indigo-nao*
cd nao_robot_control/nao_virtual_ws
catkin_make

```

##### Usage

* Kinect

  kinect/openTracker\_ws is a ROS work space
  
  run
  
  ```
  rosrun openni\_tracker openni\_tracker
  ```

  This will use openni\_tracker package to do skeleton tracking
  
* Nao robot

  nao\_virtual\_ws is a ROS work space

  run

  ```  
  roslaunch nao\_gazebo\_plugin nao\_gazebo\_plugin\_H25.launch
  ```

  In another terminal, run:
  
  ```  
  ./test
  ```  

  This script will give you 8 seconds to stand before kinect camera in a "give up" pose
  
  to calibrate.
  
  
