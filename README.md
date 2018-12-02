# icub-ds-walking


## System Requirements
Ubuntu 16.04, Gazebo7, ROS-Kinetic, YARP

## Installation
1. To avoid any issues, we recommend to start with a clean Ubuntu 16.04 installation. Then, follow the instructions in the [biped-walking-controller](https://github.com/epfl-lasa/biped-walking-controller) to install Gazebo7/YARP/iCub/gazebo-yarp-plugin etc. Run the test example to check that everything is working properly.

2. Then install ``ros-kinetic-desktop`` which includes ROS, rqt, rviz, and robot-generic libraries only.

3. Do the following steps:
* In your catkin src directory clone the repository
```
$ git clone -b nadia https://github.com/epfl-lasa/icub-ds-walking
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```
$  wstool init
$  wstool merge icub-ds-walking/dependencies.rosinstall 
$  wstool up 
```
* Query and installs all libraries and packages 
```
$ rosdep install --from-paths . --ignore-src --rosdistro kinetic 
```


