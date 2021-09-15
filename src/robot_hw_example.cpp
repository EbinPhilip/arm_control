#include "Arm_Robot_HW.h"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "robot_hw_example");
    ros::NodeHandle nh("robot_hw_example");

    Arm_Robot_HW robot;
    robot.init(nh, nh);
    controller_manager::ControllerManager cm(&robot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); // 10 Hz rate
    
    while (ros::ok())
    {
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        
        robot.read();
        cm.update(time, period);
        robot.write();
        
        rate.sleep();
    }
    return 0;
}