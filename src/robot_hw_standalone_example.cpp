#include "Arm_Robot_HW.h"

#include "ros/ros.h"

#include <map>
#include <memory>

#include <iostream>
#include <chrono>

#include <signal.h>
#include <thread>
#include <functional>
#include <cmath>

void update(const Arm_Robot_HW& controller_temp)
{
    std::cout<<"update thread started"<<std::endl;

    Arm_Robot_HW& controller = const_cast<Arm_Robot_HW&>(controller_temp);
    auto t1 = std::chrono::steady_clock::now();

    while(ros::ok())
    {
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
        -t1).count() < 100);
        t1 = std::chrono::steady_clock::now();
        controller.write();
        controller.read();
    }
}

void printState(std::vector<Joint_Properties_Ptr> servo_list)
{
    std::cout<<"display thread started"<<std::endl;
    auto t1 = std::chrono::steady_clock::now();

    while(ros::ok())
    {
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
        -t1).count() < 100);
        t1 = std::chrono::steady_clock::now();
        
        for(auto servo: servo_list)
        {
            std::cout<<("Pos: ");
            std::cout<<servo->state.position;
            std::cout<<(" degrees");
            std::cout<<(" | ");
            std::cout<<("Speed: ");
            std::cout<<servo->state.velocity;
            std::cout<<(" rpm");
            std::cout<<(" | ");
            std::cout<<("Effort: ");
            std::cout<<servo->state.effort;
            if (servo != servo_list.back())
                std::cout<<("\t|||\t");
        }
        std::cout<<std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hw_standalone_example");
    ros::NodeHandle nh("robot_hw_standalone_example");

    int n;
    std::cin>>n;

    Arm_Robot_HW hw;
    hw.init(nh, nh);

    double position1 = M_PI*0.5;
    double position2 = M_PI;

    double speed1 = 3.141593 ;
    double speed2 = 5.235988;
    auto joint1 = hw.joint_map_->operator[]("joint1");
    auto joint2 = hw.joint_map_->operator[]("joint2");
    auto joint3 = hw.joint_map_->operator[]("joint3");

    std::vector<Joint_Properties_Ptr> vec = {joint1, joint2, joint3};

    std::thread update_thread(update, std::ref(hw));
    std::thread display_thread(printState, vec);

    while (ros::ok())
    {
        joint1->command.position = joint3->command.position = position1;
        joint2->command.position = position2;

        joint1->command.velocity = joint3->command.velocity = speed1;
        joint2->command.velocity = speed2;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        joint1->command.position = joint3->command.position = position2;
        joint2->command.position = position1;

        joint1->command.velocity = joint3->command.velocity = speed2;
        joint2->command.velocity = speed1;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        ros::spinOnce();
    }

    update_thread.join();
    display_thread.join();


}