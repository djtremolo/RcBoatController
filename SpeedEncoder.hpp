#ifndef __SPEED_ENCODER_HPP__
#define __SPEED_ENCODER_HPP__

#include "Common.hpp"

void enc_initialize(void);
void enc_getData(float &rpsR, float &rpsL);
void enc_getData(float &rpsR, float &rpsL, int32_t &diff);  /*diff for debug*/
bool enc_measuringCycleElapsed();

#endif