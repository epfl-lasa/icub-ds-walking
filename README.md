## icub-ds-walking
DS-based motion planning for the iCub using the reactive omnidirectional walking controller proposed in [1] with DS learned from demonstrations with the LPV-DS approach [2] and the LAGS-DS approach [3].

### System Requirements
Ubuntu 16.04, Gazebo7, ROS-Kinetic, YARP

### Installation
To avoid any issues, we recommend to start with a clean Ubuntu 16.04 installation. 

1. Install  [biped-walking-controller](https://github.com/epfl-lasa/biped-walking-controller/tree/nadia-DS) package.
   - Checkout/clone the ``nadia-DS`` branch!
   ```bash
      $ git clone -b nadia-DS https://github.com/epfl-lasa/biped-walking-controller.git
   ```
   - Install all dependencies Gazebo7/YARP/iCub/gazebo-yarp-plugin etc (follow README of [biped-walking-controller](https://github.com/epfl-lasa/biped-walking-controller/tree/nadia-DS)):  
       **Easiest option (on clean installation):**
      - Install latest version of [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page): Eigen 3 version >=3.2.9 
      - Install Gazebo7 and libgazebo7: [installation instructions](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0) 
      - Install all yarp/iCub/gazebo-plugin libraries with [robotology-superbuild](https://github.com/robotology/robotology-superbuild)
      - Install the Gazebo Plugin [GetLinkWorldPose](https://github.com/epfl-lasa/GetLinkWorldPose.git)

   - ***Run the tests with different walking commands to check that everything is working properly.***

2. Install ``ros-kinetic-desktop`` which includes ROS, rqt, rviz, and robot-generic libraries only.

3. In your catkin src directory clone the repository
   ```bash
      $ git clone https://github.com/epfl-lasa/icub-ds-walking
   ```
   * wstool gets all other git repository dependencies, after the following steps you should see extra catkin 
   packages in your src directory.
   ```bash
      $  wstool init
      $  wstool merge icub-ds-walking/icub-ds-motion/dependencies.rosinstall && wstool up 
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
### Usage
- **Terminal 1** Start yarpserver:
   ```bash
      $ yarpserver
   ```
- **Terminal 2** (simulation) start gazebo simulator and import include the robot model (`iCub (no hands)`)
   ```bash
      $ gazebo 
   ```
- **Terminal 3** Launch roscore and visualization of CoM and DS in Rviz: (This could be in another PC)
   ```bash
      $ roslaunch icub-ds-motion icub_visualization.launch
   ```
- **Terminal 4** Run the walking controller as follows : 
   ```bash
      $ ./BipedWalkingGrasping_ROS --from ../config/BalanceWalkingController_ROS.ini
   ```
- **Terminal 5** Once the ports are open, run the yarp2ros publisher:
   ```bash
      $ rosrun yarp2ros_data_publisher yarp2ros_CoM_node --robot icubSim
   ```
   - Name of the robot should be the same as the one defined in ```~/biped-walking-controller/config/BalanceWalkingController_ROS.ini```
   
- **Terminal 5** Load the DS that you want the robot's CoM motion to follow:
   ```bash
      $ roslaunch icub-ds-motion load_DScontroller.launch 
   ```
   To define which DS you want to load you can modify the launch file:
   ```xml
   # Load DS Motion Generator
	<include file="$(find icub-ds-motion)/launch/load_lpvDS_motionGenerator.launch">
		<arg name="DS_name" value="iCub-Line-Loco"/>
	</include>
	# Example Options:
  	# - iCub-Line-Loco
  	# - iCub-Linear-Loco
  	# - iCub-C-Loco
   ```
   These are the names of the ```.yml``` files that should be stored in this folder: ```~/ds_motion_generator/config/learned_DS/lpvDS/```.

#### Testing different walking commands
We currently have 2 different ways of generating desired CoM velocity (v<sub>x</sub>, v<sub>y</sub>, w<sub>z</sub>). These types and their parameters can be defined in the config file: ``BalanceWalkingController_ROS.ini`` like so,
```
# DS Type 0: Simple linear DS, 1: Using a non-linear DS from ROS
DSType		0
```
1. Desired Velocity will be generated via a simple linear DS: ``DSType		0``   
The implemented DS is of the form <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/linear_DS.gif"> whose parameters can be defined as follows:
  ```
    # Desired Target with linear DS x [m], y [m], z [m] 
    kappa      0.2
    AttractorX 2.00
    AttractorY -1.00
    AttractorZ 0.541591
  ```  
   where:  
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/CoM.gif">: CoM position
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/attractor.gif">: Attractor (target)
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/kappa.gif">: DS gain     

   The angular velocity <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega_z.gif"> is defined with the following equation: <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega_eq.gif">, where:  
    
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/R.gif">:  Current Rotation matrix of the robot's CoM in world reference frame  
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/R_d.gif">: Desired Rotation matrix of the robot's CoM in world reference frame, computed by aligning R with the direction of motion given by the DS <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/ds_dir.gif">  
   - <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega_skew.gif">: The skew-symmetric matrix representing the angular velocity vector <img src="https://github.com/epfl-lasa/biped-walking-controller/blob/nadia-DS/img/omega.gif">  

2. Desired Velocity will be generated via a non-linear DS learned from demonstrations: ``DSType		1``. The paramaters of this DS should be defined in.. launch file

---

**References**     
> [1] Bombile, M. and Billard, A. (2017) Capture-Point based Balance and Reactive Omnidirectional Walking Controller. In proceedings of the IEEE-RAS Conference on Humanoid Robots (Humanoids), Birmigham (UK), Nov 15-17, 2017  
> [2] Figueroa, N. and Billard, A. (2018) A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning. In Proceedings of the 2nd Conference on Robot Learning (CoRL).     
> [3] Figueroa, N and Billard, A. (2019) "Locally Active Globally Stable Dynamical Systems: Theory, Learning and Experiments". In Preparation. 

**Contact**: [Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

**Acknowledgments**
This work was supported by the European Community Horizon 2020 Research and Innovation pro-
gramme, grant agreement 644727-[Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon).
