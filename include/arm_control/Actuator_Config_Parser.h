#ifndef __ACTUATOR_CONFIG_PARSER__
#define __ACTUATOR_CONFIG_PARSER__

#include "Rosparam_Config_Parser.h"
#include "Actuator_Controller.h"

typedef Rosparam_Config_Parser<Actuator_Controller_Map> Actuator_Config_Parser;

#endif