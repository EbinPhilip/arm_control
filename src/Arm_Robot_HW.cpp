#include "Arm_Robot_HW.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <memory>

using namespace XmlRpc;

Arm_Robot_HW::Arm_Robot_HW()
    : joint_map_(std::make_shared<std::map<std::string, Joint_Properties_Ptr>>()),
    transmissions_map_(std::make_shared<std::map<std::string,Transmission_Properties_Ptr>>()),
    controller_map_(std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>()),
    actuator_interface_(nullptr)
{
}

bool Arm_Robot_HW::init(ros::NodeHandle &root_nh, ros::NodeHandle &nh)
{
    XmlRpcValue value;
    nh.getParam("", value);

    Joint_Config_Parser joint_parser;
    joint_parser.parseConfig(value, joint_map_);

    Transmission_Config_Parser transmission_parser;
    transmission_parser.parseConfig(value, transmissions_map_);

    Actuator_Controller_Map controller_map = std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>();
    Actuator_Controllers_Creator creator;
    creator.parseConfig(value, controller_map);
    actuator_interface_ = std::make_shared<Actuator_Control_Interface>(controller_map);

    for(auto& it : *transmissions_map_)
    {
        it.second->registerTransmission(*this, *actuator_interface_, joint_map_);
    }

    for(auto& it : *joint_map_)
    {
        it.second->registerJointInterfaces(*this);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&pos_vel_joint_interface_);

    return true;
}

void Arm_Robot_HW::read()
{
    actuator_interface_->readState();
    actuator_to_joint_interface_.propagate();
}

void Arm_Robot_HW::write()
{
    for (auto& it : *joint_map_)
    {
        it.second->setDefaultVelocity();
    }
    joint_to_actuator_interface_.propagate();
    actuator_interface_->writeCommand();
}


hardware_interface::JointStateInterface &Arm_Robot_HW::getJointStateInterface()
{
    return joint_state_interface_;
}

hardware_interface::PositionJointInterface &Arm_Robot_HW::getPositionJointInterface()
{
    return position_joint_interface_;
}

hardware_interface::PosVelJointInterface &Arm_Robot_HW::getPosVelJointInterface()
{
    return pos_vel_joint_interface_;
}

transmission_interface::ActuatorToJointStateInterface &Arm_Robot_HW::getActuatorToJointStateInterface()
{
    return actuator_to_joint_interface_;
}

transmission_interface::JointToActuatorStateInterface &Arm_Robot_HW::getJointToActuatorStateInterface()
{
    return joint_to_actuator_interface_;
}