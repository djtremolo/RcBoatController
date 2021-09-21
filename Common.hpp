#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include "Arduino.h"
#include "stdint.h"

bool com_checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted);

#endif
