#ifndef __HAL_HPP__
#define __HAL_HPP__

#include "Common.hpp"

#define RC_INPUT_REGISTER           PIND
#define RC_DIRECTION_REGISTER       DDRD
#define RC_OUTPUT_REGISTER          PORTD

#define RC_BIT_DIRECTION            2
#define RC_BIT_THROTTLE             3


#define SS_INPUT_REGISTER           PIND
#define SS_DIRECTION_REGISTER       DDRD
#define SS_OUTPUT_REGISTER          PORTD

#define SS_BIT_SENSOR1              4
#define SS_BIT_SENSOR2              5

#endif