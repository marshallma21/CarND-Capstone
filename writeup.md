## Capstone Project - Team Fridy 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the Capstone project for the Udacity Self-Driving Car Nanodegree.
We developed software to guide a real self-driving car around a test track.
Using the Robot Operating System (ROS), we created nodes for :
- traffic light detection and classification
- trajectory planning
- control(steering, acceleration and brake)


You can find the latest version of this project on
[Github](https://github.com/marshallma21/CarND-Capstone-Friday.git)



### Beyond the Requirements

---

### Team Members

| Name | Github  | Uda email |
| --- | --- | --- |
| Ma |  |  |
|Gu|  |  |
|Wu|  |  |
|Long|  |  |
| Lingfeng Ai |  [Lingfeng Ai](https://github.com/hanxiaomax)  |  hanxiaoamx@qq.com |

---

### Contents
[toc]

---

### Project Components



#### Visualization
#### Traffic Light Detection
#### Traffic Light Classification
#### Trajectory Planner
#### Waypoint Follower
#### Controllers


-------------

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |

We are working on a fix to line up the OpenCV versions between the two.

### Trouble shooting 
#### 1.If you're seeing the following error message during `catkin_make`

```
CMake Warning at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:76 (find_package):
Could not find a package configuration file provided by "dbw_mkz_msgs" with
any of the following names:

dbw_mkz_msgsConfig.cmake
dbw_mkz_msgs-config.cmake

```

Install the following package

```
sudo apt-get install -y ros-kinetic-dbw-mkz-msgs
```

#### 2.if you encounter this error information when using workspace
```
[ 75%] Generating EusLisp code from styx_msgs/Lane.msg
Traceback (most recent call last):
  File "/opt/ros/kinetic/share/geneus/cmake/../../../lib/geneus/gen_eus.py", line 39, in <module>
    import geneus
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/geneus/__init__.py", line 32, in <module>
    from . geneus_main import *
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/geneus/geneus_main.py", line 43, in <module>
    from catkin import terminal_color
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/catkin/terminal_color.py", line 2, in <module>
    from catkin_pkg.terminal_color import *  # noqa
ImportError: No module named terminal_color
``` 
please use `pip install --upgrade  catkin_pkg_modules`