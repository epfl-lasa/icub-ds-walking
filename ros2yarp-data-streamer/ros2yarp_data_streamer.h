#ifndef ROS2YARP_DATA_STREAMER_H
#define ROS2YARP_DATA_STREAMER_H


#include <iostream>
#include <iomanip>

#include "ros/ros.h"
#include "cartesian_state_msgs/PoseTwist.h"
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

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;


class ros2yarp_data_streamer
{

	protected: 

		// ROS VARIABLES:
		// A handle to the node in ros
		ros::NodeHandle nh_;
		// Rate of the run loop
		ros::Rate loop_rate_;

		// moduleName
		std::string moduleName;


		// Subscribers:

		// Subscriber for the current object pose in the world defined by the vision system
		ros::Subscriber sub_current_object_pose_world_;
		// Subscriber for the desired object pose
		ros::Subscriber sub_desired_object_pose_world_;

		// Subscriber for the wheeled robot end-effector pose pose
		ros::Subscriber sub_wheeled_robot_endeffector_pose_world_;
		// Subscriber for the wheeled robot platform pose pose
		ros::Subscriber sub_wheeled_robot_platform_pose_world_;
		// Subscriber for the mid pc pose in world defined by mocap
		ros::Subscriber sub_mid_pc_pose_mocap_world_;

		// Subscriber for the humanoid base pose
		ros::Subscriber sub_humanoid_base_pose_world_;
		// Subscriber for the left hand end effector
		ros::Subscriber sub_humanoid_lhand_endeffector_world_;
		// Subscriber for the right hand end effector
		ros::Subscriber sub_humanoid_rhand_endeffector_world_;


		// Publishers:

		// Publisher for the reference object pose in the world frame
		ros::Publisher pub_reference_object_pose_world_;
		// Publisher for the reference object twist in the world frame
		ros::Publisher pub_reference_object_twist_world_;
		// Publisher for the desired object twist in the world frame
		ros::Publisher pub_reference_object_acceleration_world_;


		// Callbacks
		void current_object_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void desired_object_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void wheeled_robot_endeffector_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void wheeled_robot_platform_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void mid_pc_pose_mocap_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void humanoid_base_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void humanoid_lhand_endeffector_world_callback(const geometry_msgs::PoseStampedConstPtr msg);
		void humanoid_rhand_endeffector_world_callback(const geometry_msgs::PoseStampedConstPtr msg);

		// Publish method
		void publish_object_references_trajectories();


		// getter
		bool get_pose_with_axis_angle();
		bool get_object_references_trajectories();



		//
		// Declaring Yarp Buffured Ports
		BufferedPort<yarp::sig::Vector> CurObjectPosePort;
		BufferedPort<yarp::sig::Vector> DesObjectPosePort;
		BufferedPort<yarp::sig::Vector> WRobotEndEffPosePort;
		BufferedPort<yarp::sig::Vector> WRobotPlatformPosePort;
		BufferedPort<yarp::sig::Vector> WRobotMidPcPosePort;
		BufferedPort<yarp::sig::Vector> HRobotBasePosePort;
		BufferedPort<yarp::sig::Vector> HRobotLHandPosePort;
		BufferedPort<yarp::sig::Vector> HRobotRHandPosePort;

		BufferedPort<yarp::sig::Vector> Ref_ObjectTwistPort;
		BufferedPort<yarp::sig::Vector> Ref_ObjectAccelPort;

		// yarp vectors to contain the data
		yarp::sig::Vector CurrentObjectPose;
		yarp::sig::Vector DesiredObjectPose;
		yarp::sig::Vector WheeledRobotEndEffectorPose;
		yarp::sig::Vector WheeledRobotPlatformPose;
		yarp::sig::Vector WheeledRobotMidPcPose;
		yarp::sig::Vector HumanoidRobotBasePose;
		yarp::sig::Vector HumanoidRobotLHandPose;
		yarp::sig::Vector HumanoidRobotRHandPose;

		yarp::sig::Vector reference_ObjectTwist;
		yarp::sig::Vector reference_ObjectAccel;


		// Positions and orientations
		Eigen::Vector3d CurObjectPosition; 
		Eigen::Quaterniond quat_CurObjectOrienation;

		Eigen::Vector3d DesObjectPosition; 
		Eigen::Quaterniond quat_DesObjectOrienation;

		Eigen::Vector3d WRobotEndEffPosition; 
		Eigen::Quaterniond quat_WRobotEndEffOrienation;

		Eigen::Vector3d WRobotPlatformPosition; 
		Eigen::Quaterniond quat_WRobotPlatformOrienation; 

		Eigen::Vector3d WRobotMidPcPosition; 
		Eigen::Quaterniond quat_WRobotMidPcOrienation;  

		Eigen::Vector3d HRobotBasePosition;
		Eigen::Quaterniond quat_HRobotBaseOrienation; 

		Eigen::Vector3d HRobotLHandPosition; 
		Eigen::Quaterniond quat_HRobotLHandOrienation;  

		Eigen::Vector3d HRobotRHandPosition;
		Eigen::Quaterniond quat_HRobotRHandOrienation; 



	public :
		//
		//
		ros2yarp_data_streamer(ros::NodeHandle &n, double frequency,
								std::string module_name_,
								std::string topic_current_object_pose_world_,
								std::string topic_desired_object_pose_world_,
								std::string topic_wheeled_robot_endeffector_pose_world_,
								std::string topic_wheeled_robot_platform_pose_world_,
								std::string topic_mid_pc_pose_mocap_world_,
								std::string topic_humanoid_base_pose_world_,
								std::string topic_humanoid_lhand_endeffector_world_,
								std::string topic_humanoid_rhand_endeffector_world_,
								std::string topic_reference_object_twist_world_,
								std::string topic_reference_object_acceleration_world_);


								

		~ros2yarp_data_streamer();

		

		void run();

};

#endif // ROS2YARP_DATA_STREAMER_H




