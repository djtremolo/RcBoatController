#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#include "Common.hpp"



void mot_initialize();
void mot_outputUpdate();
void mot_valueSet(int idx, int16_t valueInPercent);
void mot_outputEnable(int idx, bool enable);


#endif
