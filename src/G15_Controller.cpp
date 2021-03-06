#include "G15_Controller.h"

#include <stdexcept>
#include <cmath>

G15_Controller::G15_Controller(const std::string &controller_name, const std::string &serial_port)
    : controller_name_(controller_name),
      serial_port_(serial_port),
      hal_(serial_port),
      servo_controller_(hal_)
{
    servo_controller_.begin(115200);
    hal_.delayMilliseconds(2000);
}

void G15_Controller::addServo(G15_Actuator_Properties_Ptr actuator)
{
    if (servo_controller_.ping(actuator->servo_id))
    {
        throw std::runtime_error("ping failed: " + actuator->actuator_name +
                                 " ID:" + std::to_string(actuator->servo_id));
    }
    else
    {
        servo_map_.insert(std::make_pair(actuator->actuator_name, actuator));
    }
}

void G15_Controller::readState()
{
    uint8_t read_bytes[6] = {0};
    uint16_t value = 0;

    for (auto servo : servo_map_)
    {
        if ( !servo_controller_.readInstruction(servo.second->servo_id, PRESENT_POSITION_L, read_bytes, 6) )
        {
            value = read_bytes[1] & 0x03;
            value = (value << 8) | read_bytes[0];
            servo.second->state.position =
                servo_controller_.convertFromRegisterValue(value, POSITION_MAX_REGISTER, 2*M_PI);

            value = read_bytes[3] & 0x01;
            value = (value << 8) | read_bytes[2];
            servo.second->state.velocity =
                servo_controller_.convertFromRegisterValue(value, SPEED_MAX_REGISTER, SPEED_MAX_RADIANS);
            
            value = read_bytes[5] & 0x01;
            value = (value << 8) | read_bytes[4];
            servo.second->state.effort =
                servo_controller_.convertFromRegisterValue(value, 1023, 1);
        }
    }
}

void G15_Controller::writeCommand()
{
    for (auto servo : servo_map_)
    {
        uint16_t position = servo_controller_.convertToRegisterValue(servo.second->command.position, 2*M_PI, POSITION_MAX_REGISTER);
        uint16_t velocity = servo_controller_.convertToRegisterValue(servo.second->command.velocity, SPEED_MAX_RADIANS, SPEED_MAX_REGISTER);

        uint8_t packet[] = 
            {
                (uint8_t)(position & 0xff),
                (uint8_t)(position >> 8),
                (uint8_t)(velocity & 0xff),
                (uint8_t)(velocity >> 8),
            };
        
        servo_controller_.regWriteInstruction(servo.second->servo_id, GOAL_POSITION_L, packet, 4);
    }
    servo_controller_.actionInstruction();
}

Arm_Actuator_Properties_Ptr G15_Controller::getActuator(const std::string& name)
{
    auto it = servo_map_.find(name);
    if (it != servo_map_.end())
    {
        return (it->second);
    }
    else
    {
        return nullptr;
    }
}

void G15_Controller::getActuatorNames(std::vector<std::string>& names)
{
    for (auto it : servo_map_)
    {
        names.push_back(it.first);
    }
}
