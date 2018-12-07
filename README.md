## icub-ds-walking
DS-based motion planning for the iCub using the reactive omnidirectional walking controller proposed in [1] with DS learned from demonstrations with the LPV-DS approach [2] and the LAGS-DS approach [3].

### System Requirements
Ubuntu 16.04, Gazebo7, ROS-Kinetic, YARP

### Installation
To avoid any issues, we recommend to start with a clean Ubuntu 16.04 installation. 

1. Install  [biped-walking-controller](https://github.com/epfl-lasa/biped-walking-controller/tree/nadia-DS) package to install Gazebo7/YARP/iCub/gazebo-yarp-plugin etc.
   - Checkout/clone the ``nadia-DS`` branch!
   ```bash
      $ git clone -b nadia-DS https://github.com/epfl-lasa/biped-walking-controller.git
   ```
   - Install all dependencies (follow README):
       **Easiest option (on clean installation):**
      - Install latest version of [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page): Eigen 3 version >=3.2.9 
      - Install Gazebo7 and libgazebo7: [installation instructions](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) 
      - Install all yarp/iCub/gazebo-plugin libraries with [robotology-superbuild](https://github.com/robotology/robotology-superbuild)
      - Install the Gazebo Plugin [GetLinkWorldPose](https://github.com/epfl-lasa/GetLinkWorldPose.git)

   - ***Run the tests with different walking commands to check that everything is working properly.***

2. Install ``ros-kinetic-desktop`` which includes ROS, rqt, rviz, and robot-generic libraries only.

3. In your catkin src directory clone the repository
```bash
$ git clone -b nadia https://github.com/epfl-lasa/icub-ds-walking
```
* wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
  packages in your src directory.
```bash
$  wstool init
$  wstool merge icub-ds-walking/dependencies.rosinstall && wstool up 
$  wstool merge ds_motion_generator/dependencies.rosinstall && wstool up 
```
* Query and installs all libraries and packages 
```bash
$ rosdep install --from-paths . --ignore-src --rosdistro kinetic 
```
* Compile all ros-packages
```bash
$ roscd && cd .. && catkin_make
```

**References**     
> [1] Bombile, M. and Billard, A. (2017) Capture-Point based Balance and Reactive Omnidirectional Walking Controller. In proceedings of the IEEE-RAS Conference on Humanoid Robots (Humanoids), Birmigham (UK), Nov 15-17, 2017  
> [2] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL).     
> [3] Figueroa, N and Billard, A. (2019) "Locally Active Globally Stable Dynamical Systems: Theory, Learning and Experiments". In Preparation. 

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the European Community Horizon 2020 Research and Innovation pro-
gramme, grant agreement 644727-[Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon).
