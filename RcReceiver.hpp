#ifndef __RC_RECEIVER_HPP__
#define __RC_RECEIVER_HPP__
#include "Common.hpp"


void rcr_initialize(void);

void rcr_getData(int16_t &throttlePct, int16_t &directionPct);
bool rcr_checkRadioStatus();

#endif