#ifndef __BOAT_CONTROLLER_HPP__
#define __BOAT_CONTROLLER_HPP__

#include "Common.hpp"

void ctr_initialize();
void ctr_speedControl(float& fOutR, float& fOutL, const int16_t throttleInPercent, const int16_t directionInPercent);
void ctr_speedAdjust(float& fOutR, float& fOutL, uint32_t encTicksR, uint32_t encTicksL);

#endif