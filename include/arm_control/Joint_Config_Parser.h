#ifndef __JOINT_CONFIG_PARSER_H__
#define __JOINT_CONFIG_PARSER_H__

#include "Rosparam_Config_Parser.h"
#include "Joint_Properties.h"

class Joint_Config_Parser : public Rosparam_Config_Parser<Joint_Map>
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Joint_Map map) override;
};

#endif