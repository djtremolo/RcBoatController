#include "BoatController.hpp"


void speedAdjust(int16_t& outR, int16_t& outL);
void speedControl();

void setupspeedAdjustTimer() 
{
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 8 Hz (16000000/((31249+1)*64))
  OCR1A = 31249;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 64
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
}











void speedControl()
{
  volatile uint32_t seqNo = speedSensorSeqNo;
  static uint32_t prevSeqNo = 0;

  /*check if 125ms timer tick has passed*/
  if(seqNo != prevSeqNo)
  {
    int16_t throttleInPercent = getThrottlePosition();
    int16_t directionInPercent = getSteeringDirection();
    int16_t outL = 0;
    int16_t outR = 0;


#if 0
    Serial.print(throttleInPercent);
    Serial.print(",");
    Serial.print(directionInPercent);
    Serial.println(",0,100");
#endif


#if 0
    throttleInPercent /= 5;
#endif


#if 0
    /*ZAIM*/if(throttleInPercent>5) throttleInPercent = 5; else throttleInPercent = 0;
    /*ZAIM*/directionInPercent = 0;
#endif

    if(throttleInPercent != 0)
    {
      int16_t primaryMotor = throttleInPercent;

      /*calculate value for the "another" motor*/

      int16_t secondaryMotor = map(abs(directionInPercent), 0, 100, primaryMotor, -primaryMotor);

      if(directionInPercent < 0)
      {
        /*steering left*/
        outL = secondaryMotor;
        outR = primaryMotor;
      }
      else
      {
        /*steering right*/
        outL = primaryMotor;
        outR = secondaryMotor;
      }
    }

    speedAdjust(outR, outL);

    motorValueSet(0, 0);  //ZAIM outR);
    motorValueSet(1, outL);

    motorOutputUpdate();

    prevSeqNo = seqNo;
  }

  return;
}


constexpr int motorMaxRps = 250;
constexpr float motorPctToRpsFactor = (float)motorMaxRps / 100.0;

void speedAdjust(int16_t& outR, int16_t& outL)
{
  static float adjFactor[2] = {1.0, 1.0};
  static int16_t prevReqInAbs[2] = {};
  int ch;
  const int16_t newReqInAbs[2] = {abs(outR), abs(outL)};
    
  /*
  ch: 0 = R
      1 = L
  */

  for(ch = 0; ch < 2; ch++)
  {
    /*note: this part will be triggered by 125ms (one eighth of a second). When the encoder reports one tick, it corresponds to one Revolution Per Second (RPS). Similarly, 100 ticks -> 100RPS = 6000RPM.*/
    uint32_t eights = speedSensorEighthsOfRevolution[ch];
    int16_t reqInAbs = prevReqInAbs[ch];
    float measuredRPS = (float)eights;
    float requestedRPS = (float)reqInAbs * motorPctToRpsFactor;   /*reqInAbs is in percents: 1...100. The motor max is more than 250RPS with no load but we want to keep some reserve for water resistance.*/
    float adj = 1.0;
    float newReq = (float)newReqInAbs[ch] * motorPctToRpsFactor;
    float reqFlt = (requestedRPS + newReq) / 2.0;   /*average prev and new to react smoother*/

    float refUsedInCalculations = reqFlt;//requestedRPS;

    if(eights != 0)
    {
      adj = refUsedInCalculations / measuredRPS;
    }
    else
    {
      /*no movement detected, check if movement was requested*/
      if(refUsedInCalculations > 0)
      {
        /*req>0 and eighths==0 -> stalled. Give the motor a push to get it rotating.*/
        adj = 2.0;  //((float)map(reqFlt, 0, 100, 40, 10)) / 10.0;
      }
    }

    /*limit*/
    if(adj > 3.0)
      adj = 3.0;
    else if(adj < 0.5)
      adj = 0.5;

    adjFactor[ch] = adj;

#if 0
    if(ch==1)
    {
      Serial.print(outL);
      Serial.print(",");
      Serial.print(requestedRPS);
      Serial.print(",");
      Serial.print(measuredRPS);
      Serial.print(",");
      Serial.print(adjFactor[1]);
      Serial.print(",");
      Serial.print(round(((float)outL) * adjFactor[1]));
      Serial.println(",0,10");
    }
  #endif
#if 1
    if(ch==1)
    {
      Serial.print(getBatteryLevelInPercents());
      Serial.print(",");
      Serial.print(requestedRPS);
      Serial.print(",");
      Serial.print(measuredRPS);
      Serial.print(",");
      Serial.print(adjFactor[1]*100.0);
    }
  #endif
  }




#if 0 ///////////ZAIM
  outR = (int16_t)round(((float)outR) * adjFactor[0]);
  outL = (int16_t)round(((float)outL) * adjFactor[1]);
#endif ///////////ZAIM

  Serial.print(",");
  Serial.print(outL);
  Serial.println(",0,400");




  prevReqInAbs[0] = abs(outR);
  prevReqInAbs[1] = abs(outL);

}

