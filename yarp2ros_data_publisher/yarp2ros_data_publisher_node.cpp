
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <string>
#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "yarp2ros_data_publisher/yarp2ros_data_publisher.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

int main(int argc, char *argv[]) 
{
    //
    ros::init(argc, argv, "yarp2ros_data_publisher");
    //
    ros::NodeHandle nh;
  	double frequency = 125.0;
  

  	// Connecting to yarp network
    Network yarp;

    Property params;
    params.fromCommand(argc, argv);
    //
    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "usage : yarp2ros_data_publisher --robot name (e.g. icub)\n");
        return 1;
    }

    std::string robotName   = params.find("robot").asString().c_str();

    // declaring a yarp2ros data publisher object
    yarp2ros_data_publisher yarpDataPub(nh, frequency, robotName);

    //
  	ros::Rate loop_rate(frequency);

    // 
    while(nh.ok())
    {

    	// get the CoM values
    	yarpDataPub.get_CoM_values();

    	// publish the data from the measurement
    	yarpDataPub.publish_CoM_pose();

    	ros::spinOnce();
    	loop_rate.sleep();

    }

   	yarpDataPub.closePorts();
    
    return 0;
}
