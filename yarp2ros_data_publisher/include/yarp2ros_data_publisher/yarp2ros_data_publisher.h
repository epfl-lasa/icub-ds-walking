#ifndef YARP2ROS_DATA_PUBLISHER_H
#define YARP2ROS_DATA_PUBLISHER_H


#include <iostream>
#include <iomanip>

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/os/BufferedPort.h>

using namespace std;
using namespace Eigen;
using namespace yarp::os;

typedef Matrix<double, 6, 1> Vector6d;


class yarp2ros_data_publisher
{

	protected: 

		// ROS VARIABLES:
		// A handle to the node in ros
		ros::NodeHandle nh_;
		// Rate of the run loop
		ros::Rate loop_rate_;

		// moduleName
		std::string robot_name;

		// Yarp ports to read the data from:
		Bottle              *CoMPose_values;
        BufferedPort<Bottle> CoMPose_port_In;
        Eigen::VectorXd      CoMPose_vector;

		// Publishers:
		ros::Publisher pub_CoM_pose_;	
	

	public :
		//
		yarp2ros_data_publisher(ros::NodeHandle &n, double frequency, 
								std::string robot_name_);

		~yarp2ros_data_publisher();

		// Publish methods
		void publish_CoM_pose();

		// getter
		bool get_CoM_values();

		void closePorts();

};

#endif // YARP2ROS_DATA_PUBLISHER_H
