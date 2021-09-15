#ifndef __TRANSMISSION_CONFIG_PARSER_H__
#define __TRANSMISSION_CONFIG_PARSER_H__

#include "Rosparam_Config_Parser.h"
#include "Transmission_Properties.h"

#include <pluginlib/class_loader.h>
#include <transmission_interface/transmission.h>

class Transmission_Config_Parser : public Rosparam_Config_Parser<Transmissions_Map>
{
public:
    Transmission_Config_Parser();
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Transmissions_Map map) override;
protected:
    pluginlib::ClassLoader<transmission_interface::Transmission> transmission_loader_;
};

#endif