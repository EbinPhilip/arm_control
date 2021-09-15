#include "ros/ros.h"
#include <xmlrpcpp/XmlRpcValue.h> 

#include "Actuator_Controllers_Creator.h"
#include "Actuator_Controller.h"
#include "Actuator_Control_Interface.h"


#include <iostream>

using namespace XmlRpc;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "actuator_config_example");
    ros::NodeHandle nh("actuator_config_example");

    XmlRpcValue value;

    int n;
    std::cin>>n;

    nh.getParam("", value);

    Actuator_Controller_Map controller_map = std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>();

    Actuator_Controllers_Creator creator;

    creator.parseConfig(value, controller_map);

    Actuator_Control_Interface interface(controller_map);
}