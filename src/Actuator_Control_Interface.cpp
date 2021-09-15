#include "Actuator_Control_Interface.h"

Actuator_Control_Interface::Actuator_Control_Interface(Actuator_Controller_Map controller_map)
    : controller_map_(controller_map)
{
    for (auto controller : *controller_map_)
    {
        std::vector<std::string> actuator_names;
        controller.second->getActuatorNames(actuator_names);

        for (auto actuator : actuator_names)
        {
            actuator_name_controller_map_.insert(std::make_pair(actuator, controller.second));
        }
    }
}

void Actuator_Control_Interface::readState()
{
    for (auto it : *controller_map_)
    {
        it.second->readState();
    }
}

void Actuator_Control_Interface::writeCommand()
{
    for (auto it : *controller_map_)
    {
        it.second->writeCommand();
    }
}

Arm_Actuator_Properties_Ptr Actuator_Control_Interface::getActuator(const std::string &name)
{
    auto it = actuator_name_controller_map_.find(name);
    if (it != actuator_name_controller_map_.end())
    {
        return it->second->getActuator(name);
    }
    else
    {
        return nullptr;
    }
}

void Actuator_Control_Interface::getActuatorNames(std::vector<std::string> &names)
{
    for (auto it : actuator_name_controller_map_)
    {
        names.push_back(it.first);
    }
}