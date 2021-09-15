#ifndef __ARM_ROBOT_HW_H__
#define __ARM_ROBOT_HW_H__

#include "Actuator_Controllers_Creator.h"
#include "Actuator_Controller.h"
#include "Actuator_Control_Interface.h"
#include "Arm_Actuator_Properties.h"

#include "Joint_Config_Parser.h"
#include "Joint_Properties.h"
#include "Transmission_Config_Parser.h"
#include "Transmission_Properties.h"

#include "Hardware_Interface_Accessors.h"

#include "ros/ros.h"

#include <hardware_interface/robot_hw.h>

class Arm_Robot_HW : public hardware_interface::RobotHW,
                     public Joint_Interfaces_Accessor,
                     public Transmission_Interfaces_Accessor

{
public:
    Arm_Robot_HW();

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &nh) override;

    virtual void read();
    virtual void write();

    virtual hardware_interface::JointStateInterface &getJointStateInterface() override;
    virtual hardware_interface::PositionJointInterface &getPositionJointInterface() override;
    virtual hardware_interface::PosVelJointInterface &getPosVelJointInterface() override;

    virtual transmission_interface::ActuatorToJointStateInterface &getActuatorToJointStateInterface() override;
    virtual transmission_interface::JointToActuatorStateInterface &getJointToActuatorStateInterface() override;

    Joint_Map joint_map_;
protected:
    
    Transmissions_Map transmissions_map_;
    Actuator_Controller_Map controller_map_;

    Actuator_Control_Interface_Ptr actuator_interface_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::PosVelJointInterface pos_vel_joint_interface_;

    transmission_interface::ActuatorToJointStateInterface actuator_to_joint_interface_;
    transmission_interface::JointToActuatorStateInterface joint_to_actuator_interface_;
};

#endif