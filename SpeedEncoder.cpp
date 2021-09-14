#include "SpeedEncoder.hpp"




void isrSpeedSensor0();
void isrSpeedSensor1();
uint32_t speedSensorRawValueGet(const int ch);
uint32_t speedSensorRpmAbsGet(const int ch);


volatile uint32_t speedSensorTicks[2];
volatile uint32_t speedSensorEighthsOfRevolution[2];
volatile uint32_t speedSensorSeqNo;   /*incremented each time the RPS values have been updated. Used for synchronizing the speedAdjust.*/
volatile uint32_t sgTimerCntr = 0;




ISR(TIMER1_COMPA_vect) 
{
  int ch;
  volatile uint32_t ctr[2];
  static uint32_t prevCtr[2] = {};

  ctr[0] = speedSensorTicks[0];
  ctr[1] = speedSensorTicks[1];

  sgTimerCntr++;

  for(ch = 0; ch < 2; ch++)
  {
    speedSensorEighthsOfRevolution[ch] = ctr[ch] - prevCtr[ch];
  }

  prevCtr[0] = ctr[0];
  prevCtr[1] = ctr[1];
  
  /*inform speedAdjust (it must be updated at less than 125ms interval to be able to catch changes. Recommended 60ms or less.*/
  speedSensorSeqNo++;
}





void sensorContextInitialize(void)
{
  speedSensorTicks[0] = 0;
  speedSensorTicks[1] = 0;
  speedSensorSeqNo = 0;
}


void setupSpeedSensor()
{
  sensorContextInitialize();

  pinMode(SS_BIT_SENSOR1, INPUT_PULLUP);
  pinMode(SS_BIT_SENSOR2, INPUT_PULLUP);

  /*Assign interrupt service routines to both ports. TODO: change to non-arduino later, as the isr uses raw IO*/
  attachPCINT(digitalPinToPCINT(SS_BIT_SENSOR1), isrSpeedSensor0, CHANGE);
  attachPCINT(digitalPinToPCINT(SS_BIT_SENSOR2), isrSpeedSensor1, CHANGE);

}

void isrSpeedSensor0()
{
  speedSensorTicks[0]++;
}

void isrSpeedSensor1()
{
  speedSensorTicks[1]++;
}
