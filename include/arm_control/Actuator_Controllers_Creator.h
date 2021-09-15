#ifndef __ACTUATOR_CONTROLLERS_CREATOR_H__
#define __ACTUATOR_CONTROLLERS_CREATOR_H__

#include "Actuator_Config_Parser.h"

class Actuator_Controllers_Creator : public Actuator_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) override;
protected:
    void _createActuatorControllers(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map);
    void _addActuators(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map);

    void _createEntityFromConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map);
    XmlRpc::XmlRpcValue& _getEntityListFromConfig(XmlRpc::XmlRpcValue& config,const std::string& entity_name);
};

#endif