#ifndef __ACTUATOR_CONTROL_INTERFACE_H__
#define __ACTUATOR_CONTROL_INTERFACE_H__

#include "Arm_Actuator_Properties.h"
#include "Actuator_Controller.h"

#include <string>
#include <vector>
#include <memory>
#include <map>

class Arm_Hardware;

class Actuator_Control_Interface : public Actuator_Controller
{
public:
    Actuator_Control_Interface(std::shared_ptr< std::map<std::string, Actuator_Controller_Ptr> >);

    virtual void readState() override;
    virtual void writeCommand() override;

    virtual Arm_Actuator_Properties_Ptr getActuator(const std::string&) override;
    virtual void getActuatorNames(std::vector<std::string>&) override;

protected:
    Actuator_Controller_Map controller_map_;
    std::map<std::string, Actuator_Controller_Ptr> actuator_name_controller_map_;
};

typedef std::shared_ptr<Actuator_Control_Interface> Actuator_Control_Interface_Ptr;

#endif