
#include "ros/ros.h"
#include "ros2yarp_data_streamer.h"

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ros2yarp_data_streamer_node");

	ros::NodeHandle nh;
	double frequency = 100.0;

	// Parameters
	std::string module_name;
	std::string topic_current_object_pose_world;
	std::string topic_desired_object_pose_world;
	std::string topic_wheeled_robot_endeffector_pose_world;
	std::string topic_wheeled_robot_platform_pose_world;
	std::string topic_mid_pc_pose_mocap_world;
	std::string topic_humanoid_base_pose_world;
	std::string topic_humanoid_lhand_endeffector_world;
	std::string topic_humanoid_rhand_endeffector_world;
	std::string topic_reference_object_twist_world;
	std::string topic_reference_object_acceleration_world;

	// LOADING PARAMETERS FROM THE ROS SERVER
	// Module name
	if (!nh.getParam("module_name", module_name)) {
		ROS_ERROR("Couldn't retrieve the module name of the ros to yarp data streamer.");
		return -1;
	}

	// Topic names
	// -------------
	if (!nh.getParam("topic_current_object_pose_world", topic_current_object_pose_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the current object pose in world.");
		return -1;
	}

	if (!nh.getParam("topic_desired_object_pose_world", topic_desired_object_pose_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the desired object pose in world.");
		return -1;
	}

	if (!nh.getParam("topic_wheeled_robot_endeffector_pose_world", topic_wheeled_robot_endeffector_pose_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the wheeled robot endeffector pose in world.");
		return -1;
	}

	if (!nh.getParam("topic_wheeled_robot_platform_pose_world", topic_wheeled_robot_platform_pose_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the wheeled robot platform pose in world.");
		return -1;
	}

	if (!nh.getParam("topic_mid_pc_pose_mocap_world", topic_mid_pc_pose_mocap_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the mid pc pose mocap in world.");
		return -1;
	}

	if (!nh.getParam("topic_humanoid_base_pose_world", topic_humanoid_base_pose_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the humanoid base pose in world.");
		return -1;
	}

	if (!nh.getParam("topic_humanoid_lhand_endeffector_world", topic_humanoid_lhand_endeffector_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the humanoid lhand endeffector in world.");
		return -1;
	}

	if (!nh.getParam("topic_humanoid_rhand_endeffector_world", topic_humanoid_rhand_endeffector_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the humanoid rhand endeffector in world.");
		return -1;
	}

	//
	if (!nh.getParam("topic_reference_object_twist_world", topic_reference_object_twist_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the reference object twist in world.");
		return -1;
	}

	if (!nh.getParam("topic_reference_object_acceleration_world", topic_reference_object_acceleration_world)) {
		ROS_ERROR("Couldn't retrieve the topic name for the reference object acceleration in world.");
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
	ros2yarp_data_streamer ros_to_yarp_DataStreamer(nh,
													frequency,
													module_name,
													topic_current_object_pose_world,
													topic_desired_object_pose_world,
													topic_wheeled_robot_endeffector_pose_world,
													topic_wheeled_robot_platform_pose_world,
													topic_mid_pc_pose_mocap_world,
													topic_humanoid_base_pose_world,
													topic_humanoid_lhand_endeffector_world,
													topic_humanoid_rhand_endeffector_world,
													topic_reference_object_twist_world,
													topic_reference_object_acceleration_world);


	ros_to_yarp_DataStreamer.run();



	return 0;
}