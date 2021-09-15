#ifndef __ROSPARAM_CONFIG_PARSER_H__
#define __ROSPARAM_CONFIG_PARSER_H__

#include <xmlrpcpp/XmlRpcValue.h> 

template<typename Entity_Map_Ptr>
class Rosparam_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Entity_Map_Ptr map) = 0;
};

#endif