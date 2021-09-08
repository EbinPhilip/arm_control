#include "G15_Controller.h"
#include "G15_Actuator_Properties.h"
#include "Actuator_Control_Interface.h"

#include <map>
#include <memory>

#include <iostream>
#include <chrono>

#include <signal.h>
#include <thread>
#include <functional>

bool stop_execution = false;

void signalHandler(int s)
{
    stop_execution = true;
}

void updateServo(const Actuator_Control_Interface& controller_temp)
{
    std::cout<<"update thread started"<<std::endl;

    Actuator_Control_Interface& controller = const_cast<Actuator_Control_Interface&>(controller_temp);
    auto t1 = std::chrono::steady_clock::now();

    while(!stop_execution)
    {
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()
        -t1).count() < 100);
        t1 = std::chrono::steady_clock::now();
        controller.writeCommand();
        controller.readState();
    }
}

void printState(std::vector<G15_Actuator_Properties_Ptr> servo_list)
{
    std::cout<<"display thread started"<<std::endl;
    auto t1 = std::chrono::steady_clock::now();

    while(!stop_execution)
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

int main(void)
{
    signal(SIGINT, signalHandler);

    G15_Actuator_Properties_Ptr g15_1 = std::make_shared<G15_Actuator_Properties>("g15_1", 1);
    G15_Actuator_Properties_Ptr g15_2 = std::make_shared<G15_Actuator_Properties>("g15_2", 2);
    G15_Actuator_Properties_Ptr g15_3 = std::make_shared<G15_Actuator_Properties>("g15_3", 3);

    std::shared_ptr<G15_Controller> controller = std::make_shared<G15_Controller>("example_controller", "/dev/ttyUSB0");

    controller->addServo(g15_1);
    controller->addServo(g15_2);
    controller->addServo(g15_3);

    double position1 = 90.0;
    double position2 = 180.0;

    double speed1 = 30.0;
    double speed2 = 30.0;

    g15_1->command.position = g15_3->command.position = position1;
    g15_2->command.position = position2;

    g15_1->command.velocity = g15_3->command.velocity = speed1;
    g15_2->command.velocity = speed2;

    std::vector<G15_Actuator_Properties_Ptr> servo_list;
    servo_list.push_back(g15_1);
    servo_list.push_back(g15_2);
    servo_list.push_back(g15_3);

    std::shared_ptr<std::map<std::string, Actuator_Controller_Ptr>> actuator_ptr = 
    std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>();

    actuator_ptr->insert(std::make_pair("g15", controller));

    Actuator_Control_Interface interface(actuator_ptr);

    std::thread update_thread(updateServo, std::ref(interface));
    std::thread display_thread(printState, servo_list);

    while(!stop_execution)
    {
        g15_1->command.position = g15_3->command.position = position1;
        g15_2->command.position = position2;

        g15_1->command.velocity = g15_3->command.velocity = speed1;
        g15_2->command.velocity = speed2;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        g15_1->command.position = g15_3->command.position = position2;
        g15_2->command.position = position1;

        g15_1->command.velocity = g15_3->command.velocity = speed2;
        g15_2->command.velocity = speed1;

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    update_thread.join();
    display_thread.join();

}