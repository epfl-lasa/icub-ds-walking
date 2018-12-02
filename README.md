## icub-ds-walking
DS-based motion planning for the iCub using the reactive omnidirectional walking controller proposed in [1] with DS learned from demonstrations with the LPV-DS approach [2] and the LAGS-DS approach [3].

### System Requirements
Ubuntu 16.04, Gazebo7, ROS-Kinetic, YARP

### Installation
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


**References**     
> [1] Bombile, M. and Billard, A. (2017) Capture-Point based Balance and Reactive Omnidirectional Walking Controller. In proceedings of the IEEE-RAS Conference on Humanoid Robots (Humanoids), Birmigham (UK), Nov 15-17, 2017  
> [2] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL).     
> [3] Figueroa, N and Billard, A. (2019) "Locally Active Globally Stable Dynamical Systems: Theory, Learning and Experiments". In Preparation. 

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the European Communitys Horizon 2020 Research and Innovation pro-
gramme ICT-23-2014, grant agreement 644727-[Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon).
