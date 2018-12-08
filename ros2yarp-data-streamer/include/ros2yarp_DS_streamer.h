#ifndef ROS2YARP_DS_STREAMER_H
#define ROS2YARP_DS_STREAMER_H


#include <iostream>
#include <iomanip>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
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

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;


class ros2yarp_DS_streamer
{

	protected: 

		// ROS VARIABLES:
		// A handle to the node in ros
		ros::NodeHandle nh_;
		// Rate of the run loop
		ros::Rate loop_rate_;

		// moduleName
		std::string robot_name_;


		// Subscribers:
		ros::Subscriber sub_desired_DS_CoM_Velocity_;

		// Publishers:
		// ...

		// Callbacks
		void desired_DS_CoM_velocity_callback(const geometry_msgs::TwistConstPtr msg);

		//
		// Declaring Yarp Buffured Ports
		BufferedPort<yarp::sig::Vector> DesCoMVelocityPort_;
	    std::string DesCoMVelocityPortName_;

		// yarp vectors to contain the data
		yarp::sig::Vector DesiredCoMVelocity_;

		// Desired CoM Velocity
		geometry_msgs::Twist DS_CoM_Velocity; 

	public :
		//
		//
		ros2yarp_DS_streamer(ros::NodeHandle &n, double frequency,
								std::string module_name,
								std::string topic_desired_DS_CoM_velocity);

								

		~ros2yarp_DS_streamer();
		

		void run();

};

#endif // ROS2YARP_DS_STREAMER_H




