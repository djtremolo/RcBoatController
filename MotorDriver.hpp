#ifndef __MOTOR_DRIVER_HPP__
#define __MOTOR_DRIVER_HPP__

#include "Common.hpp"

typedef enum
{
  MOTOR_COAST,
  MOTOR_FORWARD,
  MOTOR_REVERSE,
  MOTOR_BRAKE
} motorDrive_t;

typedef volatile struct
{
  bool resetCycle;

  uint8_t activeDutyValue[2];   
  uint8_t passiveDutyValue[2];

  uint16_t activeWidth; /*pre-calculated for ISR: ticks to stay active*/
  uint16_t totalWidth;  /*pre-calculated for ISR: ticks to rest*/
} pwmDataForIsr_t;

typedef struct
{
  bool enable;
  int16_t value;    /*free to be written at any time*/

  /*two pwm data sets. The non-active one is used for preparing for next PWM cycle. 
  After the data is prepared, the activeIndex is changed to point to new values.*/
  int activePwmDataForIsrIndex;
  pwmDataForIsr_t pwmData[2];
} motorContext_t;


#endif
