
#include "ros2yarp_DS_streamer.h"

using namespace std;
using namespace Eigen;
using namespace yarp::os;


// ------------------
ros2yarp_DS_streamer::ros2yarp_DS_streamer(ros::NodeHandle &n, double frequency,
												std::string robot_name,
												std::string topic_desired_DS_CoM_velocity,
												std::string topic_desired_DS_CoM_attractor)	: nh_(n), loop_rate_(frequency)
																										, robot_name_(robot_name)
																										, DesiredCoMVelocity_(3)
																										, DesiredCoMAttractor_(2)
{
	// Subscribers
  	//	
	sub_desired_DS_CoM_Velocity_ = nh_.subscribe(topic_desired_DS_CoM_velocity, 10,
                                   &ros2yarp_DS_streamer::desired_DS_CoM_velocity_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());
	sub_desired_DS_CoM_Attractor_ = nh_.subscribe(topic_desired_DS_CoM_attractor, 10,
                                   &ros2yarp_DS_streamer::desired_DS_CoM_attractor_callback, this,
                                   ros::TransportHints().reliable().tcpNoDelay());

	// opening the ports
	DesCoMVelocityPortName_ = "/" + robot_name_ + "/DesiredCoMVelocity:o";
	DesCoMVelocityPort_.open(DesCoMVelocityPortName_.c_str());


	// opening the ports
	DesCoMAttractorPortName_ = "/" + robot_name_ + "/DesiredCoMAttractor:o";
	DesCoMAttractorPort_.open(DesCoMAttractorPortName_.c_str());

	// initilize the pose vectors to zero
	DesiredCoMVelocity_ = 0.0;
	DesiredCoMAttractor_ = 0.0;
	
}

ros2yarp_DS_streamer::~ros2yarp_DS_streamer()
{
	DesCoMVelocityPort_.close();	
}

//
// current_object_pose_world_callback
// -------------------------------------
void ros2yarp_DS_streamer::desired_DS_CoM_velocity_callback(const geometry_msgs::TwistConstPtr msg) 
{
	// position
	DesiredCoMVelocity_[0] = msg->linear.x;
	DesiredCoMVelocity_[1] = msg->linear.y;
	DesiredCoMVelocity_[2] = msg->linear.z;

	// ROS_INFO_STREAM("DS desired velocity v_x: " << DesiredCoMVelocity_[0] << " v_y: "<< DesiredCoMVelocity_[1]  << " v_z: "<< DesiredCoMVelocity_[2] << std::endl);

	// orientation (for when I implemented the learned orientation dynamics)
	// DesiredCoMVelocity_[3] = msg->angular.x;	
	// DesiredCoMVelocity_[4] = msg->angular.y;	
	// DesiredCoMVelocity_[5] = msg->angular.z;	

}

void ros2yarp_DS_streamer::desired_DS_CoM_attractor_callback(const geometry_msgs::PointStampedConstPtr msg) 
{
	// position
	DesiredCoMAttractor_[0] = msg->point.x;
	DesiredCoMAttractor_[1] = msg->point.y;

}


//
void ros2yarp_DS_streamer::run() 
{

	ROS_INFO("Running the ros2yarp_DS_streamer  loop ..............");

	bool stream = true;

	while (nh_.ok()) 
	{

		//
		yarp::sig::Vector &output_DesCoMVelocity = DesCoMVelocityPort_.prepare();
		yarp::sig::Vector &output_DesCoMAttractor = DesCoMAttractorPort_.prepare();

		// assinging the pose values to the ports
		// ---------------------------------------
		output_DesCoMVelocity  		=	DesiredCoMVelocity_; 
		output_DesCoMAttractor  		=	DesiredCoMAttractor_; 

		// write the data to the port
		// --------------------------
		DesCoMVelocityPort_.write();
		DesCoMAttractorPort_.write();

		ros::spinOnce();
		loop_rate_.sleep();

	}

	// Interrupt the streaming when exiting the loop
	// --------------------------
	DesCoMVelocityPort_.interrupt();
	DesCoMAttractorPort_.interrupt();

}