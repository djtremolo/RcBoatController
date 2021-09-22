#include "Controller.hpp"

static float fMap(float x, float in_min, float in_max, float out_min, float out_max);

typedef struct
{
  float prevRequestedRps;
  float adjustmentAvg;
} speedAdjustContext_t;

void speedAdjust(float &fOut, const float &encRps, speedAdjustContext_t &ctx);


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



const float pctToRps(const float &pct)
{
  constexpr int motorMaxRpm = 15000;    /*roughly measured in free spin with fresh batteries, lowered with certain margin to be close enough*/
  constexpr int motorMaxRps = motorMaxRpm / 60;
  constexpr float motorPctToRpsFactor = (float)motorMaxRps / 100.0;

  return (const float)(pct * motorPctToRpsFactor);
}


/*note: this function should be called only when enc module has got new data (timer cycle = 10ms)*/
void speedAdjust(float &fOut, const float &encRps, speedAdjustContext_t &ctx)
{
  constexpr int adjAvgWindowSize = 10;
  constexpr int adjAvgPreferNewValues = 1;  /*this must be in between 1...adjAvgWindowSize*/
  constexpr float adjMaxJumpFactorSpeedingUp = 0.4;
  constexpr float adjMaxJumpFactorSlowingDown = 0.5;

  const float requestedRps = pctToRps(fOut);
  const float measuredRps = encRps;

  const float targetedRpsAbs = abs(ctx.prevRequestedRps);

  if(targetedRpsAbs >= 5)
  {
    float adj;

    /*calculate new adjustment over previous cycle*/
    if(measuredRps >= 5)
    {
      adj = targetedRpsAbs / measuredRps;

      if(adj > 1.0) 
        adj *= 3;
      else if(adj < 1.0)
        adj *= 0.4;
    }
    else
    {
      /*stalled? give motor a push of 4x, which will be ramped up by limiting process*/
      adj = 4.0;
    }

    /*limit new adjustment to avoid too jumpy behaviour*/
    const float adjMin = ctx.adjustmentAvg * (1.0 - adjMaxJumpFactorSlowingDown);
    const float adjMax = ctx.adjustmentAvg * (1.0 + adjMaxJumpFactorSpeedingUp);

    adj = constrain(adj, adjMin, adjMax);

    /*filter adjustment factor by averaging it*/
    float adjAvgSum = (ctx.adjustmentAvg * (adjAvgWindowSize-adjAvgPreferNewValues)) + (adj * adjAvgPreferNewValues);
    ctx.adjustmentAvg = adjAvgSum / adjAvgWindowSize;
  }
  else
  {
    ctx.adjustmentAvg = 1.0;  /*idling*/
  }

  /*remember the requested value for the next round*/
  ctx.prevRequestedRps = requestedRps;

  /*calculate new output value in pct*/
  float tmp = fOut * ctx.adjustmentAvg;

  /*make sure we are not exceeding the limits*/
  tmp = constrain(tmp, -100, 100);

  /*expose the adjusted value*/
  fOut = tmp;
}

/*note: this function should be called only when enc module has got new data (timer cycle = 10ms)*/
void ctr_speedAdjust(float &fOutR, float &fOutL, const float &encRpsR, const float &encRpsL)
{
  static speedAdjustContext_t ctxR = {};
  static speedAdjustContext_t ctxL = {};

  speedAdjust(fOutR, encRpsR, ctxR);
  speedAdjust(fOutL, encRpsL, ctxL);
}
