#ifndef __BOAT_CONTROLLER_HPP__
#define __BOAT_CONTROLLER_HPP__

#include "Common.hpp"

void ctr_initialize();
void ctr_speedControl(int16_t& outR, int16_t& outL, const int16_t throttleInPercent, const int16_t directionInPercent);
void ctr_speedAdjust(int16_t& outR, int16_t& outL, uint32_t encTicksR, uint32_t encTicksL);

#endif