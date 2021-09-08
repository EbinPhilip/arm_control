#ifndef __ARM_ACTUATOR_PROPERTIES_H__
#define __ARM_ACTUATOR_PROPERTIES_H__

#include "Control_Properties.h"

#include <transmission_interface/transmission.h>

#include <string>
#include <memory>

const std::string g15_actuator_string = "g15";

enum class Actuator_Type : uint8_t
{
    unknown_actuator = 0,
    g15 = 1
};

Actuator_Type getTypeFromActuatorString(const std::string &actuatorString)
{
    if (actuatorString == g15_actuator_string)
    {
        return Actuator_Type::g15;
    }
    else
    {
        return Actuator_Type::unknown_actuator;
    }
}

struct Arm_Actuator_Properties : public Controllable_Entity
{
    std::string actuator_name;
    Actuator_Type actuator_type;
};

typedef std::shared_ptr<Arm_Actuator_Properties> Arm_Actuator_Properties_Ptr;

#endif