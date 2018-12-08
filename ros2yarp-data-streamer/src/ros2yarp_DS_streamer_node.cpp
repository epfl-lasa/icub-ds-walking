
#include "ros/ros.h"
#include "ros2yarp_DS_streamer.h"

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ros2yarp_DS_streamer_node");

	ros::NodeHandle nh;
	double frequency = 125.0;

	// Parameters
	std::string robot_name  = "icubSim";
	std::string topic_desired_DS_CoM_velocity = "/ds_cmd_vel";
	std::string topic_desired_DS_CoM_attractor = "/ds1/DS/target";


	// LOADING PARAMETERS FROM THE ROS SERVER
	// Module name
	if (!nh.getParam("robot_name", robot_name)) {
		ROS_ERROR("Couldn't retrieve the robot name of the ros to yarp data streamer.");
		return -1;
	}

	// Topic names
	// -------------
	if (!nh.getParam("topic_desired_DS_CoM_velocity", topic_desired_DS_CoM_velocity)) {
		ROS_ERROR("Couldn't retrieve the topic name for the desired CoM velocity from DS.");
		return -1;
	}

	// -------------
	if (!nh.getParam("topic_desired_DS_CoM_attractor", topic_desired_DS_CoM_attractor)) {
		ROS_ERROR("Couldn't retrieve the topic name for the desired CoM attractor from DS.");
		return -1;
	}

	// opening connecting to yarp network
	//initialize the network
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5)) 
    {
        ROS_ERROR("YARP network is not available");
        return -1;
    }

	// creating the streamer
	ros2yarp_DS_streamer ros_to_yarp_DS_streamer(nh, frequency, robot_name, topic_desired_DS_CoM_velocity, topic_desired_DS_CoM_attractor);


	ros_to_yarp_DS_streamer.run();



	return 0;
}