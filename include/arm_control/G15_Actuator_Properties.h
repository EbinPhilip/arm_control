#ifndef __G15_ACTUATOR_PROPERTIES_H__
#define __G15_ACTUATOR_PROPERTIES_H__

#include "Arm_Actuator_Properties.h"

#include <memory>

struct G15_Actuator_Properties : public Arm_Actuator_Properties
{
    G15_Actuator_Properties(const std::string& name, uint8_t id_number)
        : servo_id(id_number),
          last_sent(0),
          error_code(0),
          error_count(0)
    {
        actuator_name =  name;
        actuator_type = Actuator_Type::g15;
    }
    uint8_t servo_id;

    unsigned long last_sent;
    bool error_status;
    uint8_t error_code;
    uint8_t error_count;
};

typedef std::shared_ptr<G15_Actuator_Properties> G15_Actuator_Properties_Ptr;

#endif