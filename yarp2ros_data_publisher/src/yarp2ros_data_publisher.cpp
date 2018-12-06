
#include "yarp2ros_data_publisher/yarp2ros_data_publisher.h"

using namespace std;
using namespace Eigen;
using namespace yarp::os;


// ------------------
yarp2ros_data_publisher::yarp2ros_data_publisher(ros::NodeHandle &n, double frequency, std::string robot_name_)	: nh_(n) 
																												, loop_rate_(frequency)
																												, robot_name(robot_name_)
																										
{
	// names of topics to be published
	std::string topic_CoM_pose_ = robot_name + "_CoM_pose";

	// Publishers:
	pub_CoM_pose_ 		         = nh_.advertise<geometry_msgs::PoseStamped>(topic_CoM_pose_, 10);


	// Opening Ports and establish connection
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //-----------
    CoMPose_port_In.open("/CoMPose_readPort");

    // remote port (on the robot side)
    //--------------------------------
    std::string CoM_portName="/";
	            CoM_portName += robot_name;
	            CoM_portName += "/CoMPose/analog:o";

	// connection to the ports
	// ------------------------
	Network::connect(CoM_portName.c_str(),	 CoMPose_port_In.getName().c_str());
       
	// Reading of all ports and initilize vectors of data
	CoMPose_values  	= CoMPose_port_In.read();
       
    // Resizing of the measurement vectors    
	CoMPose_vector.resize(CoMPose_values->size());		
	CoMPose_vector.setZero();
   
}

yarp2ros_data_publisher::~yarp2ros_data_publisher()
{
    
    CoMPose_port_In.close();
	
}


bool yarp2ros_data_publisher::get_CoM_values()
{
	CoMPose_values = CoMPose_port_In.read();

    for (int i=0; i<CoMPose_values->size(); i++) 
        CoMPose_vector(i)  = CoMPose_values->get(i).asDouble();

	return true;
}

// publishing of the reference trajectories
void yarp2ros_data_publisher::publish_CoM_pose()
{
		
	geometry_msgs::PoseStamped  msg_CoM_pose;

	msg_CoM_pose.header.stamp       = ros::Time::now();
	msg_CoM_pose.header.frame_id    = "CoM_link";
	msg_CoM_pose.pose.position.x    = CoMPose_vector(0);
	msg_CoM_pose.pose.position.y    = CoMPose_vector(1);
	msg_CoM_pose.pose.position.z    = CoMPose_vector(2);
	msg_CoM_pose.pose.orientation.x = CoMPose_vector(3);
	msg_CoM_pose.pose.orientation.y = CoMPose_vector(4);
	msg_CoM_pose.pose.orientation.z = CoMPose_vector(5);
	msg_CoM_pose.pose.orientation.w = CoMPose_vector(6);

	// publishing the messages
	pub_CoM_pose_.publish(msg_CoM_pose);

}


void yarp2ros_data_publisher::closePorts()
{
	CoMPose_port_In.close();
}