#ifndef __SPEED_ENCODER_HPP__
#define __SPEED_ENCODER_HPP__

#include "Common.hpp"

void enc_initialize(void);
void enc_getData(uint32_t &chRight, uint32_t &chLeft);
bool enc_125msElapsed();

#endif