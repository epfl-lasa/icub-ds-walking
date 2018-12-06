
#include "ros2yarp_data_streamer.h"

using namespace std;
using namespace Eigen;
using namespace yarp::os;


// ------------------
ros2yarp_data_streamer::ros2yarp_data_streamer(ros::NodeHandle &n, double frequency,
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
												std::string topic_reference_object_acceleration_world_)	: nh_(n), loop_rate_(frequency)
																										, moduleName(module_name_)
																										, CurrentObjectPose(7)
																										, DesiredObjectPose(7)
																										, WheeledRobotEndEffectorPose(7)
																										, WheeledRobotPlatformPose(7)
																										, WheeledRobotMidPcPose(7)
																										, HumanoidRobotBasePose(7)
																										, HumanoidRobotLHandPose(7)
																										, HumanoidRobotRHandPose(7)
																										, reference_ObjectTwist(6)
																										, reference_ObjectAccel(6)
{
	// Subscribers
  	//	
	sub_current_object_pose_world_ = nh_.subscribe(topic_current_object_pose_world_, 10,
                                   &ros2yarp_data_streamer::current_object_pose_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_desired_object_pose_world_ = nh_.subscribe(topic_desired_object_pose_world_, 10,
                                   &ros2yarp_data_streamer::desired_object_pose_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_wheeled_robot_endeffector_pose_world_ = nh_.subscribe(topic_wheeled_robot_endeffector_pose_world_, 10,
                                   &ros2yarp_data_streamer::wheeled_robot_endeffector_pose_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_wheeled_robot_platform_pose_world_ = nh_.subscribe(topic_wheeled_robot_platform_pose_world_, 10,
                                   &ros2yarp_data_streamer::wheeled_robot_platform_pose_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_mid_pc_pose_mocap_world_ = nh_.subscribe(topic_mid_pc_pose_mocap_world_, 10,
                                   &ros2yarp_data_streamer::mid_pc_pose_mocap_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_humanoid_base_pose_world_ = nh_.subscribe(topic_humanoid_base_pose_world_, 10,
                                   &ros2yarp_data_streamer::humanoid_base_pose_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_humanoid_lhand_endeffector_world_ = nh_.subscribe(topic_humanoid_lhand_endeffector_world_, 10,
                                   &ros2yarp_data_streamer::humanoid_lhand_endeffector_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	sub_humanoid_rhand_endeffector_world_ = nh_.subscribe(topic_humanoid_rhand_endeffector_world_, 10,
                                   &ros2yarp_data_streamer::humanoid_rhand_endeffector_world_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	// Publishers:
	pub_reference_object_twist_world_ = nh_.advertise<geometry_msgs::TwistStamped>(topic_reference_object_twist_world_, 5);
	pub_reference_object_acceleration_world_  = nh_.advertise<geometry_msgs::TwistStamped>(topic_reference_object_acceleration_world_, 5);


	// opening the port
	// create the port names
	std::string CurObjectPosePortName 	 	= "/" + moduleName + "/CurrentObjectPose:o";
	std::string DesObjectPosePortName  	 	= "/" + moduleName + "/DesiredObjectPose:o";
	std::string WRobotEndEffPosePortName 	= "/" + moduleName + "/WRobotEndEffPose:o";
	std::string WRobotPlatformPosePortName	= "/" + moduleName + "/WRobotPlatformPose:o";
	std::string WRobotMidPcPosePortName 	= "/" + moduleName + "/WRobotMidPcPose:o";
	std::string HRobotBasePosePortName 		= "/" + moduleName + "/HRobotBasePose:o";
	std::string HRobotLHandPosePortName 	= "/" + moduleName + "/HRobotLeftHandPose:o";
	std::string HRobotRHandPosePortName 	= "/" + moduleName + "/HRobotRightHandPose:o";
	// 
	Ref_ObjectTwistPortName	= "/" + moduleName + "/ReferenceObjectTwist:o";
	Ref_ObjectAccelPortName = "/" + moduleName + "/ReferenceObjectAccel:o";


	// opening the ports
	CurObjectPosePort.open(CurObjectPosePortName.c_str());
	DesObjectPosePort.open(DesObjectPosePortName.c_str());
	WRobotEndEffPosePort.open(WRobotEndEffPosePortName.c_str());
	WRobotPlatformPosePort.open(WRobotPlatformPosePortName.c_str());
	WRobotMidPcPosePort.open(WRobotMidPcPosePortName.c_str());
	HRobotBasePosePort.open(HRobotBasePosePortName.c_str());
	HRobotLHandPosePort.open(HRobotLHandPosePortName.c_str());
	HRobotRHandPosePort.open(HRobotRHandPosePortName.c_str());
	// 
	Ref_ObjectTwistPort.open(Ref_ObjectTwistPortName.c_str());
	Ref_ObjectAccelPort.open(Ref_ObjectAccelPortName.c_str());


	// initilize the pose vectors to zero
	CurrentObjectPose = 0.0;
	DesiredObjectPose = 0.0;
	WheeledRobotEndEffectorPose = 0.0;
	WheeledRobotPlatformPose = 0.0;
	WheeledRobotMidPcPose 	= 0.0;
	HumanoidRobotBasePose 	= 0.0;
	HumanoidRobotLHandPose 	= 0.0;
	HumanoidRobotRHandPose 	= 0.0;

	//
	reference_ObjectTwist = 0.0;
	reference_ObjectAccel = 0.0;

	
}

ros2yarp_data_streamer::~ros2yarp_data_streamer()
{
	CurObjectPosePort.close();
	DesObjectPosePort.close();
	WRobotEndEffPosePort.close();
	WRobotPlatformPosePort.close();
	WRobotMidPcPosePort.close();
	HRobotBasePosePort.close();
	HRobotLHandPosePort.close();
	HRobotRHandPosePort.close();

	Ref_ObjectTwistPort.close();
	Ref_ObjectAccelPort.close();
	
}

//
// current_object_pose_world_callback
// -------------------------------------
void ros2yarp_data_streamer::current_object_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg) 
{
	// position
	CurObjectPosition[0] = msg->pose.position.x;
	CurObjectPosition[1] = msg->pose.position.y;
	CurObjectPosition[2] = msg->pose.position.z;

	// orientation
	quat_CurObjectOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;

}

//
void ros2yarp_data_streamer::desired_object_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	DesObjectPosition[0] = msg->pose.position.x;
	DesObjectPosition[1] = msg->pose.position.y;
	DesObjectPosition[2] = msg->pose.position.z;

	// orientation
	quat_DesObjectOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}
void ros2yarp_data_streamer::wheeled_robot_endeffector_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	WRobotEndEffPosition[0] = msg->pose.position.x;
	WRobotEndEffPosition[1] = msg->pose.position.y;
	WRobotEndEffPosition[2] = msg->pose.position.z;

	// orientation
	quat_WRobotEndEffOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}
void ros2yarp_data_streamer::wheeled_robot_platform_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	WRobotPlatformPosition[0] = msg->pose.position.x;
	WRobotPlatformPosition[1] = msg->pose.position.y;
	WRobotPlatformPosition[2] = msg->pose.position.z;

	// orientation
	quat_WRobotEndEffOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}
void ros2yarp_data_streamer::mid_pc_pose_mocap_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	WRobotMidPcPosition[0] = msg->pose.position.x;
	WRobotMidPcPosition[1] = msg->pose.position.y;
	WRobotMidPcPosition[2] = msg->pose.position.z;

	// orientation
	quat_CurObjectOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}
void ros2yarp_data_streamer::humanoid_base_pose_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	HRobotBasePosition[0] = msg->pose.position.x;
	HRobotBasePosition[1] = msg->pose.position.y;
	HRobotBasePosition[2] = msg->pose.position.z;

	// orientation
	quat_HRobotBaseOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}
void ros2yarp_data_streamer::humanoid_lhand_endeffector_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	HRobotLHandPosition[0] = msg->pose.position.x;
	HRobotLHandPosition[1] = msg->pose.position.y;
	HRobotLHandPosition[2] = msg->pose.position.z;

	// orientation
	quat_HRobotLHandOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}
void ros2yarp_data_streamer::humanoid_rhand_endeffector_world_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
	// position
	HRobotRHandPosition[0] = msg->pose.position.x;
	HRobotRHandPosition[1] = msg->pose.position.y;
	HRobotRHandPosition[2] = msg->pose.position.z;

	// orientation
	quat_HRobotRHandOrienation.coeffs()<< msg->pose.orientation.x,
										msg->pose.orientation.y,
										msg->pose.orientation.z,
										msg->pose.orientation.w;
}


// get pose with Axis Angle

bool ros2yarp_data_streamer::get_pose_with_axis_angle()
{
	//	
    CurrentObjectPose.setSubvector(0, CurObjectPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_CurObjectOrienation);
    CurrentObjectPose[3] = CurObjectOrienation.axis()(0);
    CurrentObjectPose[4] = CurObjectOrienation.axis()(1);
    CurrentObjectPose[5] = CurObjectOrienation.axis()(2);
    CurrentObjectPose[6] = CurObjectOrienation.angle();
    //	
    DesiredObjectPose.setSubvector(0, DesObjectPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_DesObjectOrienation);
    DesiredObjectPose[3] = CurObjectOrienation.axis()(0);
    DesiredObjectPose[4] = CurObjectOrienation.axis()(1);
    DesiredObjectPose[5] = CurObjectOrienation.axis()(2);
    DesiredObjectPose[6] = CurObjectOrienation.angle();
    //	
    WheeledRobotEndEffectorPose.setSubvector(0, WRobotEndEffPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_WRobotEndEffOrienation);
    WheeledRobotEndEffectorPose[3] = WRobotEndEffOrienation.axis()(0);
    WheeledRobotEndEffectorPose[4] = WRobotEndEffOrienation.axis()(1);
    WheeledRobotEndEffectorPose[5] = WRobotEndEffOrienation.axis()(2);
    WheeledRobotEndEffectorPose[6] = WRobotEndEffOrienation.angle();
    //	
    WheeledRobotPlatformPose.setSubvector(0, WRobotPlatformPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_WRobotPlatformOrienation);
    WheeledRobotPlatformPose[3] = WRobotPlatformOrienation.axis()(0);
    WheeledRobotPlatformPose[4] = WRobotPlatformOrienation.axis()(1);
    WheeledRobotPlatformPose[5] = WRobotPlatformOrienation.axis()(2);
    WheeledRobotPlatformPose[6] = WRobotPlatformOrienation.angle();
    //	
    WheeledRobotMidPcPose.setSubvector(0, WRobotMidPcPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_WRobotMidPcOrienation);
    WheeledRobotMidPcPose[3] = WRobotMidPcOrienation.axis()(0);
    WheeledRobotMidPcPose[4] = WRobotMidPcOrienation.axis()(1);
    WheeledRobotMidPcPose[5] = WRobotMidPcOrienation.axis()(2);
    WheeledRobotMidPcPose[6] = WRobotMidPcOrienation.angle();
    //	
    HumanoidRobotBasePose.setSubvector(0, HRobotBasePosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_HRobotBaseOrienation);
    HumanoidRobotBasePose[3] = HRobotBaseOrienation.axis()(0);
    HumanoidRobotBasePose[4] = HRobotBaseOrienation.axis()(1);
    HumanoidRobotBasePose[5] = HRobotBaseOrienation.axis()(2);
    HumanoidRobotBasePose[6] = HRobotBaseOrienation.angle();
    //	
    HumanoidRobotLHandPose.setSubvector(0, HRobotLHandPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_HRobotLHandOrienation);
    HumanoidRobotLHandPose[3] = HRobotLHandOrienation.axis()(0);
    HumanoidRobotLHandPose[4] = HRobotLHandOrienation.axis()(1);
    HumanoidRobotLHandPose[5] = HRobotLHandOrienation.axis()(2);
    HumanoidRobotLHandPose[6] = HRobotLHandOrienation.angle();
    //	
    HumanoidRobotRHandPose.setSubvector(0, HRobotRHandPosition);
    Eigen::AngleAxisd CurObjectOrienation(quat_HRobotRHandOrienation);
    HumanoidRobotRHandPose[3] = HRobotRHandOrienation.axis()(0);
    HumanoidRobotRHandPose[4] = HRobotRHandOrienation.axis()(1);
    HumanoidRobotRHandPose[5] = HRobotRHandOrienation.axis()(2);
    HumanoidRobotRHandPose[6] = HRobotRHandOrienation.angle();

	return true;
}

// 
bool ros2yarp_data_streamer::get_object_references_trajectories()
{
	// from a DS to be designed
	reference_ObjectTwist  		= 0.0;  // meantime
	reference_ObjectAccel 		= 0.0;  // meantime

	return true;
}


// publishing of the reference trajectories
void ros2yarp_data_streamer::publish_object_references_trajectories()
{
		
	
	geometry_msgs::Twist reference_object_twist_wld;
	reference_object_twist_wld.linear.x  = reference_ObjectTwist(0); // New added
	reference_object_twist_wld.linear.y  = reference_ObjectTwist(1);
	reference_object_twist_wld.linear.z  = reference_ObjectTwist(2);
	reference_object_twist_wld.angular.x = reference_ObjectTwist(3);
	reference_object_twist_wld.angular.y = reference_ObjectTwist(4);
	reference_object_twist_wld.angular.z = reference_ObjectTwist(5);

	geometry_msgs::Twist reference_object_accel_wld;
	reference_object_accel_wld.linear.x  = reference_ObjectAccel(0);
	reference_object_accel_wld.linear.y  = reference_ObjectAccel(1);
	reference_object_accel_wld.linear.z  = reference_ObjectAccel(2);
	reference_object_accel_wld.angular.x = reference_ObjectAccel(3);
	reference_object_accel_wld.angular.y = reference_ObjectAccel(4);
	reference_object_accel_wld.angular.z = reference_ObjectAccel(5);

	// publishing the messages
	pub_reference_object_twist_world_.publish(reference_object_twist_wld);
	pub_reference_object_acceleration_world_.publish(reference_object_accel_wld);

}

//
void ros2yarp_data_streamer::run() 
{

	ROS_INFO("Running the ros2yarp_data_streamer  loop ..............");

	bool stream = true;

	while (nh_.ok()) 
	{

		//
		yarp::sig::Vector &output_CurObjectPose 		= CurObjectPosePort.prepare();
		yarp::sig::Vector &output_DesObjectPose 		= DesObjectPosePort.prepare();
		yarp::sig::Vector &output_WRobotEndEffPose 		= WRobotEndEffPosePort.prepare();
		yarp::sig::Vector &output_WRobotPlatformPose 	= WRobotPlatformPosePort.prepare();
		yarp::sig::Vector &output_WRobotMidPcPose 		= WRobotMidPcPosePort.prepare();
		yarp::sig::Vector &output_HRobotBasePose 		= HRobotBasePosePort.prepare();
		yarp::sig::Vector &output_HRobotLHandPose 		= HRobotLHandPosePort.prepare();
		yarp::sig::Vector &output_HRobotRHandPose 		= HRobotRHandPosePort.prepare();

		yarp::sig::Vector &output_RefObjectTwist 		= Ref_ObjectTwistPort.prepare();
		yarp::sig::Vector &output_RefObjectAccel 		= Ref_ObjectAccelPort.prepare();


		// getting the poses with axis angle representation of the orientation
		if(!(stream = get_pose_with_axis_angle()))
		{
			ROS_INFO("Error while getting the pose with axis angle ");
			break;
		}

		// getting the reference trajectories of the object
		if(!(stream = stream && get_object_references_trajectories()))
		{
			ROS_INFO("Failed to get the object reference trajectories ");
			break;
		}

		// publish the object reference trajectories
		publish_object_references_trajectories();


		// assinging the pose values to the ports
		// ---------------------------------------
		output_CurObjectPose  		=	CurrentObjectPose; 
		output_DesObjectPose  		=	DesiredObjectPose;
		output_WRobotEndEffPose  	=	WheeledRobotEndEffectorPose;
		output_WRobotPlatformPose  	=	WheeledRobotPlatformPose;
		output_WRobotMidPcPose  	=	WheeledRobotMidPcPose;
		output_HRobotBasePose  		=	HumanoidRobotBasePose;
		output_HRobotLHandPose  	=	HumanoidRobotLHandPose;
		output_HRobotRHandPose  	=	HumanoidRobotRHandPose;

		output_RefObjectTwist		= 	reference_ObjectTwist;
		output_RefObjectAccel 		=  	reference_ObjectAccel;

		// write the data to the port
		// --------------------------
		CurObjectPosePort.write();
		DesObjectPosePort.write();
		WRobotEndEffPosePort.write();
		WRobotPlatformPosePort.write();
		WRobotMidPcPosePort.write();
		HRobotBasePosePort.write();
		HRobotLHandPosePort.write();
		HRobotRHandPosePort.write();

		Ref_ObjectTwistPort.write();
		Ref_ObjectAccelPort.write();

		// publishing visualization/debugging info
		// publish_debuggings_signals();

		ros::spinOnce();
		loop_rate_.sleep();

	}

	// Interrupt the streaming when exiting the loop
	// --------------------------
	CurObjectPosePort.interrupt();
	DesObjectPosePort.interrupt();
	WRobotEndEffPosePort.interrupt();
	WRobotPlatformPosePort.interrupt();
	WRobotMidPcPosePort.interrupt();
	HRobotBasePosePort.interrupt();
	HRobotLHandPosePort.interrupt();
	HRobotRHandPosePort.interrupt();

	Ref_ObjectTwistPort.interrupt();
	Ref_ObjectAccelPort.interrupt();


}