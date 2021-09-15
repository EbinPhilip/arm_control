#ifndef __G15_CONTROLLER_CONFIG_PARSER_H__
#define __G15_CONTROLLER_CONFIG_PARSER_H__

#include "Actuator_Config_Parser.h"

class G15_Controller_Config_Parser : public Actuator_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map controller_map) override;
protected:
    std::string _getControllerName(XmlRpc::XmlRpcValue::iterator it);
    std::string _getControllerPort(XmlRpc::XmlRpcValue::iterator it);
    unsigned int _getControllerBaudRate(XmlRpc::XmlRpcValue::iterator it);
    uint8_t _getControllerID(XmlRpc::XmlRpcValue::iterator it);
};

#endif