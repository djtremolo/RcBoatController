#include "SpeedEncoder.hpp"
#include "PinChangeInterrupt.h"


#define SS_INPUT_REGISTER           PIND
#define SS_DIRECTION_REGISTER       DDRD
#define SS_OUTPUT_REGISTER          PORTD

#define SS_BIT_SENSOR1              4
#define SS_BIT_SENSOR2              5


static void isrSpeedSensor0();
static void isrSpeedSensor1();

volatile uint32_t isrSensorTickArray[2];
volatile uint32_t eighthsOfRevolution[2];
volatile uint32_t seqNo;   /*incremented each time the RPS values have been updated. Used for synchronizing the control loop.*/
volatile uint32_t sgTimerCntr = 0;

void enc_getData(uint32_t &chRight, uint32_t &chLeft)
{
  chRight = eighthsOfRevolution[0];
  chLeft = eighthsOfRevolution[1];
}

ISR(TIMER1_COMPA_vect) 
{
  int ch;
  volatile uint32_t ctr[2];
  static uint32_t prevCtr[2] = {};

  ctr[0] = isrSensorTickArray[0];
  ctr[1] = isrSensorTickArray[1];

  sgTimerCntr++;

  for(ch = 0; ch < 2; ch++)
  {
    eighthsOfRevolution[ch] = ctr[ch] - prevCtr[ch];
  }

  prevCtr[0] = ctr[0];
  prevCtr[1] = ctr[1];
  
  /*inform ctr_speedAdjust (it must be updated at less than 125ms interval to be able to catch changes. Recommended 60ms or less.*/
  seqNo++;
}

bool enc_125msElapsed()
{
  bool ret = false;
  static uint32_t prevSeqNo = 0;

  if(seqNo != prevSeqNo)
  {
    prevSeqNo = seqNo;
    ret = true;
  }
  return ret;
}


void enc_initialize(void)
{
  isrSensorTickArray[0] = 0;
  isrSensorTickArray[1] = 0;
  eighthsOfRevolution[0] = 0;
  eighthsOfRevolution[1] = 0;
  seqNo = 0;

  pinMode(SS_BIT_SENSOR1, INPUT_PULLUP);
  pinMode(SS_BIT_SENSOR2, INPUT_PULLUP);

  /*Assign interrupt service routines to both ports.*/

  cli();  /*disable*/
  attachPCINT(digitalPinToPCINT(SS_BIT_SENSOR1), isrSpeedSensor0, CHANGE);
  attachPCINT(digitalPinToPCINT(SS_BIT_SENSOR2), isrSpeedSensor1, CHANGE);
  sei();  /*enable*/
}

static void isrSpeedSensor0()
{
  isrSensorTickArray[0]++;
}

static void isrSpeedSensor1()
{
  isrSensorTickArray[1]++;
}
