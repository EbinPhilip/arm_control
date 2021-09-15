#ifndef __ACTUATOR_CONTROLLER_H__
#define __ACTUATOR_CONTROLLER_H__

#include "Arm_Actuator_Properties.h"

#include <string>
#include <vector>
#include <memory>
#include <map>

class Actuator_Controller
{
public:
    virtual void readState() = 0;
    virtual void writeCommand() = 0;

    virtual Arm_Actuator_Properties_Ptr getActuator(const std::string&) = 0;
    virtual void getActuatorNames(std::vector<std::string>&) = 0;
};

typedef std::shared_ptr<Actuator_Controller> Actuator_Controller_Ptr;
typedef std::shared_ptr< std::map<std::string, Actuator_Controller_Ptr> > Actuator_Controller_Map;

#endif