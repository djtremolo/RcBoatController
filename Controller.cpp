#include "Controller.hpp"

static float fMap(float x, float in_min, float in_max, float out_min, float out_max);

void controlCycleTimerSetup();

void controlCycleTimerSetup()
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





void ctr_initialize()
{
  controlCycleTimerSetup();
}


static float fMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*note: this function should be called only at 125ms timer cycle*/
void ctr_speedControl(float& fOutR, float& fOutL, const int16_t throttleInPercent, const int16_t directionInPercent)
{
  if(throttleInPercent != 0)
  {
    float primaryMotor = (float)throttleInPercent;

    /*calculate value for the "another" motor*/

    float secondaryMotor = fMap(abs(directionInPercent), 0, 100, primaryMotor, -primaryMotor);

    if(directionInPercent < 0)
    {
      /*steering left*/
      fOutL = secondaryMotor;
      fOutR = primaryMotor;
    }
    else
    {
      /*steering right*/
      fOutL = primaryMotor;
      fOutR = secondaryMotor;
    }
  }
  else
  {
    fOutL = fOutR = 0.0;
  }

  return;
}



/*note: this function should be called only at 125ms timer cycle*/
void ctr_speedAdjust(float& fOutR, float& fOutL, const uint32_t encTicksR, const uint32_t encTicksL)
{
  static float adjFactor[2] = {1.0, 1.0};
  static int16_t prevReqInAbs[2] = {};

  const float newReqInAbs[2] = {abs(fOutR), abs(fOutL)};
  const uint32_t encTicks[2] = {encTicksR, encTicksL};

  constexpr int motorMaxRps = 250;    /*roughly measured in free spin with fresh batteries, lowered with certain margin to be close enough*/
  constexpr float motorPctToRpsFactor = (float)motorMaxRps / 100.0;

  /*
  ch: 0 = R
      1 = L
  */

  for(int ch = 0; ch < 2; ch++)
  {
    /*note: this part will be triggered by 125ms (one eighth of a second). When the encoder reports one tick, it corresponds to one Revolution Per Second (RPS). Similarly, 100 ticks -> 100RPS = 6000RPM.*/
    uint32_t eights = encTicks[ch];
    int16_t reqInAbs = prevReqInAbs[ch];
    float measuredRPS = (float)eights;
    float requestedRPS = (float)reqInAbs * motorPctToRpsFactor;   /*reqInAbs is in percents: 1...100. The motor max is more than 250RPS with no load but we want to keep some reserve for water resistance.*/
    float adj = 1.0;
    float newReq = newReqInAbs[ch] * motorPctToRpsFactor;
    float reqFlt = (requestedRPS + newReq) / 2.0;   /*average prev and new to react smoother*/

    float refUsedInCalculations = (1?requestedRPS:reqFlt);    /*0-> filtered input, 1->direct input*/

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
        adj = 2.0;
      }
    }

    /*limit*/
    if(adj > 5.0)
      adj = 5.0;
    else if(adj < 0.5)
      adj = 0.5;

    adjFactor[ch] = adj;
  }

  fOutR *= adjFactor[0];
  fOutL *= adjFactor[1];

  prevReqInAbs[0] = abs(fOutR);
  prevReqInAbs[1] = abs(fOutL);
}

