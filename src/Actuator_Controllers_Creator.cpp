#include "Actuator_Controllers_Creator.h"
#include "G15_Actuator_Config_Parser.h"
#include "G15_Controller_Config_Parser.h"

#include <memory>
#include <string>
#include <stdexcept>

using namespace XmlRpc;

std::shared_ptr<Actuator_Config_Parser> getConfigParser(const std::string& name)
{
    if (name == "g15_servo")
    {
        return std::make_shared<G15_Actuator_Config_Parser>();
    }
    else if (name == "g15_actuator_controller")
    {
        return std::make_shared<G15_Controller_Config_Parser>();
    }
    else
    {
        throw std::runtime_error("unknown config type: "+name);
    }
}

void Actuator_Controllers_Creator::parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("invalid actuator/controller config structure!");
    }

    _createActuatorControllers(config, controller_map);
    _addActuators(config, controller_map);
}

void Actuator_Controllers_Creator::_createActuatorControllers(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
    auto controllers_list = _getEntityListFromConfig(config, "actuator_controllers");
    _createEntityFromConfig(controllers_list, controller_map);
}

void Actuator_Controllers_Creator::_addActuators(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
    auto actuators_list = _getEntityListFromConfig(config, "actuators");
    _createEntityFromConfig(actuators_list, controller_map);
}

void Actuator_Controllers_Creator::_createEntityFromConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeArray)
    {
        throw std::runtime_error("invalid actuator/controller config structure!");
    }
    for (int i = 0; i<config.size(); ++i)
    {
        for(auto actuator_type_it = config[i].begin(); 
            actuator_type_it != config[i].end(); ++actuator_type_it)
        {
            auto actuator_creator = getConfigParser(actuator_type_it->first);
            actuator_creator->parseConfig(actuator_type_it->second, controller_map);
        }
    }
}

XmlRpcValue& Actuator_Controllers_Creator::_getEntityListFromConfig(XmlRpc::XmlRpcValue& config,const std::string& entity_name)
{
    if (!config.hasMember(entity_name))
    {
        throw std::runtime_error(entity_name+" not found!");
    }
    XmlRpcValue& controllers_list = config[entity_name];
    
    return controllers_list;
}


