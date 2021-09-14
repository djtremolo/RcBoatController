#include "HAL.hpp"
#include "PinChangeInterrupt.h"



inline void setMotorPinsISR(const int idx, volatile uint8_t &val0, volatile uint8_t &val1) __attribute__((always_inline));
void setMotorPinsISR(const int idx, volatile uint8_t &val0, volatile uint8_t &val1)
{
  //will fail if idx >= 2!

  int pinIdx = idx*2;

  if(val0 == HIGH) 
  {
    PORTC |= 0x01 << pinIdx;
  }
  else
  {
    PORTC &= ~(0x01 << pinIdx);
  }

  if(val1 == HIGH) 
  {
    PORTC |= 0x02 << pinIdx;
  }
  else
  {
    PORTC &= ~(0x02 << pinIdx);
  }
}



