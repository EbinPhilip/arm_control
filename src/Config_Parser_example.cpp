#include "Actuator_Controllers_Creator.h"
#include "Actuator_Controller.h"
#include "Actuator_Control_Interface.h"

#include "Joint_Config_Parser.h"
#include "Transmission_Config_Parser.h"

#include "ros/ros.h"
#include <xmlrpcpp/XmlRpcValue.h> 

#include <iostream>

using namespace XmlRpc;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "config_parser_example");
    ros::NodeHandle nh("config_parser_example");

    XmlRpcValue value;

    int n;
    std::cin>>n;

    nh.getParam("", value);

    Joint_Config_Parser joint_parser;
    Joint_Map joint_map = std::make_shared<std::map<std::string, Joint_Properties_Ptr>>();
    joint_parser.parseConfig(value, joint_map);

    Transmission_Config_Parser transmission_parser;
    Transmissions_Map transmissions_map = std::make_shared<std::map<std::string, Transmission_Properties_Ptr>>();
    transmission_parser.parseConfig(value, transmissions_map);

    // Actuator_Controller_Map controller_map = std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>();
    // Actuator_Controllers_Creator creator;
    // creator.parseConfig(value, controller_map);

    // Actuator_Control_Interface interface(controller_map);

}

