#include "Controller.hpp"

static float fMap(float x, float in_min, float in_max, float out_min, float out_max);


void ctr_initialize()
{
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



#if 1
const float pctToRps(const float &pct)
{
  constexpr int motorMaxRpm = 15000;    /*roughly measured in free spin with fresh batteries, lowered with certain margin to be close enough*/
  constexpr int motorMaxRps = motorMaxRpm / 60;
  constexpr float motorPctToRpsFactor = (float)motorMaxRps / 100.0;

  return (const float)(pct * motorPctToRpsFactor);
}

typedef struct
{
  float requestedPct[2];
  float requestedRps[2];
  float measuredRps[2];
  float adjustmentAvg[2];  
} speedAdjustContext_t;


/*note: this function should be called only at 125ms timer cycle*/
void ctr_speedAdjust(float &fOutR, float &fOutL, const float &encRpsR, const float &encRpsL)
{
  constexpr int adjAvgWindowSize = 10;
  constexpr int adjAvgPreferNewValues = 1;  /*this must be in between 1...adjAvgWindowSize*/
  constexpr float adjMaxJumpFactorSpeedingUp = 0.5;
  constexpr float adjMaxJumpFactorSlowingDown = 0.5;

  static float prevRequestedRps[2] = {0, 0};
  static float adjustmentAvg[2] = {0, 0};

  const float requestedRps[2] = {pctToRps(fOutR), pctToRps(fOutL)};
  const float measuredRps[2] = {encRpsR, encRpsL};

  for(int ch=0; ch<2; ch++)
  {
    const float targetedRpsAbs = abs(prevRequestedRps[ch]);

    if(targetedRpsAbs >= 5)
    {
      float adj;

      /*calculate new adjustment over previous cycle*/
      if(measuredRps[ch] >= 5)
      {
        adj = targetedRpsAbs / measuredRps[ch];

        if(adj > 1.0) 
          adj *= 2;
        else if(adj < 1.0) 
          adj *= 0.3;
      }
      else
      {
        /*stalled? give motor a push of 4x, which will be ramped up by limiting process*/
        adj = 4.0;
      }

      /*limit new adjustment to avoid too jumpy behaviour*/
      const float adjMin = adjustmentAvg[ch] * (1.0 - adjMaxJumpFactorSlowingDown);
      const float adjMax = adjustmentAvg[ch] * (1.0 + adjMaxJumpFactorSpeedingUp);
      if(adj > adjMax)
      {
        adj = adjMax;
      }
      else if(adj < adjMin)
      {
        adj = adjMin;
      }

      /*filter adjustment factor by averaging it*/
      float adjAvgSum = (adjustmentAvg[ch] * (adjAvgWindowSize-adjAvgPreferNewValues)) + (adj * adjAvgPreferNewValues);
      adjustmentAvg[ch] = adjAvgSum / adjAvgWindowSize;
    }
    else
    {
      adjustmentAvg[ch] = 1.0;  /*idling*/
    }
  }

  /*remember the requested value for the next round*/
  prevRequestedRps[0] = requestedRps[0];
  prevRequestedRps[1] = requestedRps[1];

  /*calculate new output values in pct*/
  fOutR = fOutR * adjustmentAvg[0];
  fOutL = fOutL * adjustmentAvg[1];
}

#else

/*note: this function should be called only at 125ms timer cycle*/
void ctr_speedAdjust(float& fOutR, float& fOutL, const uint32_t encTicksR, const uint32_t encTicksL)
{
  constexpr int adjWindowSize = 10;
  static float adjFactor[2] = {1.0, 1.0};
  static float adjFactorSum[2] = {0.0, 0.0};
  static float adjFactorPrev[2] = {0.0, 0.0};
  static int16_t prevReqInAbs[2] = {};
  static float prevOutR = 0.0;
  static float prevOutL = 0.0;

  static int preAvgFeed = adjWindowSize;

  const float newReqInAbs[2] = {abs(prevOutR), abs(prevOutL)};
  const uint32_t encTicks[2] = {encTicksR, encTicksL};

  constexpr int motorMaxRps = 250;    /*roughly measured in free spin with fresh batteries, lowered with certain margin to be close enough*/
  constexpr float motorPctToRpsFactor = (float)motorMaxRps / 100.0;

  /*
  ch: 0 = R
      1 = L
  */

  for(int ch = 0; ch < 2; ch++)
  {
    uint32_t eights = encTicks[ch];
    int16_t reqInAbs = prevReqInAbs[ch];
    float measuredRPS = (float)(eights * 5);
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

    if(preAvgFeed == 0)
    {
      adjFactorSum[ch] -= adjFactorPrev[ch];
    }
    adjFactorSum[ch] += adj;

    adjFactorPrev[ch] = adj;

    adjFactor[ch] = adjFactorSum[ch] / (float)adjWindowSize;
  }


  float tr = prevOutR * adjFactor[0];
  float tl = prevOutL * adjFactor[1];

  prevReqInAbs[0] = abs(prevOutR);
  prevReqInAbs[1] = abs(prevOutL);

  prevOutR = fOutR;
  prevOutL = fOutL;

  fOutR = tr;
  fOutL = tl;

  if(preAvgFeed > 0)
  {
    preAvgFeed--;
  }
}

#endif
