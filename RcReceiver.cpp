#include "RcReceiver.hpp"





#define RC_INPUT_REGISTER           PIND
#define RC_DIRECTION_REGISTER       DDRD
#define RC_OUTPUT_REGISTER          PORTD

#define RC_BIT_DIRECTION            2
#define RC_BIT_THROTTLE             3

constexpr int rcFilterBufferSize = 7;


typedef volatile struct 
{
  bool signalOk;
  bool risingEdgeSeen;
  bool fallingEdgeSeen;
  uint32_t prevRisingTs;
  uint32_t cycleWidth;
  uint32_t pulseWidthArray[rcFilterBufferSize];
  int bufIndex;
} radioControlContext_t;


radioControlContext_t rcContext[2];
uint32_t rcCntr = 0;
uint32_t pulseCounter[2] = {0};


void contextInitialize(void);

void setupRadioControlInput();

void isrRcIncomingPulse(radioControlContext_t* c, int pin, const uint32_t ts);
void isrDirectionSignalChange();
void isrThrottleSignalChange();
bool getRawData(uint32_t ch, uint32_t *pulseWidth);

int16_t getSteeringDirection();
int16_t getThrottlePosition();


void contextInitialize()
{
  for(int ch=0; ch<2; ch++)
  {
    radioControlContext_t* c = &(rcContext[ch]);
    memset((void*)c, 0, sizeof(radioControlContext_t));
  }
}

void isrRcIncomingPulse(const int ch, const uint8_t pinMask, const uint32_t ts)
{
  bool rising = ((PIND & pinMask) == 0 ? false : true);
  radioControlContext_t* c = &(rcContext[ch]);

  rcCntr++;

  if(rising)
  {
    if(c->signalOk)
    {
      /*calculate cycle width*/
      c->cycleWidth = ts - c->prevRisingTs;
    }

    /*remember context information*/
    c->prevRisingTs = ts;
    c->risingEdgeSeen = true;
  }
  else
  {
    if(c->signalOk)
    {
      /*calculate pulse width*/
      c->pulseWidthArray[c->bufIndex] = ts - c->prevRisingTs;
      c->bufIndex = (c->bufIndex < (rcFilterBufferSize-1) ? c->bufIndex + 1 : 0);
    }

    /*remember context information*/
    c->fallingEdgeSeen = true;
  }

  /*allow measurement only when both edge timestamps have been captured and widths are accepted*/
  if(c->risingEdgeSeen && c->fallingEdgeSeen)
  {
    /*everything looks good*/
    pulseCounter[ch]++;

    c->signalOk = true;
  }
}

void isrDirectionSignalChange()
{
  const uint32_t ts = micros();
  constexpr uint8_t pinMask = 1 << RC_BIT_DIRECTION;
  isrRcIncomingPulse(0, pinMask, ts);
}

void isrThrottleSignalChange()
{
  const uint32_t ts = micros();
  constexpr uint8_t pinMask = 1 << RC_BIT_THROTTLE;
  isrRcIncomingPulse(1, pinMask, ts);
}

uint32_t getFilteredPulseWidth(radioControlContext_t *c)
{
  int i;
  uint32_t sum = 0;
  uint32_t pwMax = 0;
  uint32_t pwMin = UINT32_MAX;

  for(i = 0; i < rcFilterBufferSize; i++)
  {
    uint32_t newPw = c->pulseWidthArray[i];
    sum += newPw;
    if(newPw > pwMax) pwMax = newPw;
    if(newPw < pwMin) pwMin = newPw;
  }

  /*remove biggest and smallest*/
  sum -= pwMax;
  sum -= pwMin;

  /*calculate avg for N-2*/
  return round((float)sum / (rcFilterBufferSize-2));
}

bool getRawData(uint32_t ch, uint32_t *pulseWidth)
{
  bool ret = false;

  if(ch < 2)
  {
    radioControlContext_t *c = &(rcContext[ch]);
    uint32_t pwAvg;

    pwAvg = getFilteredPulseWidth(c);

    if(c->signalOk
        && com_checkRange(c->cycleWidth, 15000, 18000)
        && com_checkRange(pwAvg, 0, c->cycleWidth)
    )
    {
      /*TODO: the reading should be atomic*/
      *pulseWidth = pwAvg;
      ret = true;
    }
  }

  return ret;
}

int16_t getSteeringDirection()
{
  int16_t steeringDirection = 0;
  uint32_t pw;

  if(getRawData(0, &pw))
  {
    int32_t dir = (int32_t)map(pw, 1110, 2064, -100, 100);

    if(abs(dir) < 5) dir = 0;
    else if(dir > 100) dir = 100;
    else if(dir < -100) dir = -100;

    steeringDirection = (int16_t)dir;
  }

  return steeringDirection;
}

int16_t getThrottlePosition()
{
  int16_t throttlePosition = 0;
  uint32_t pw, cw;

  if(getRawData(1, &pw))
  {
    //min:1136
    //max:2088
    //zero:1572

    int32_t tp;
    
    if(pw < 1572)
    {
      tp = (int32_t)map(pw, 1136, 1572, -50, 0);
    }
    else
    {
      tp = (int32_t)map(pw, 1572, 2088, 0, 100);
    }

    if(abs(tp) < 5) tp = 0;
    else if(tp > 100) tp = 100;
    else if(tp < -100) tp = -100;

    throttlePosition = (int16_t)tp;
  }

  return throttlePosition;
}


void rcr_getData(int16_t &throttlePct, int16_t &directionPct)
{
  throttlePct = getThrottlePosition();
  directionPct = getSteeringDirection();
}

void rcr_initialize(void)
{
  contextInitialize();

  pinMode(RC_BIT_DIRECTION, INPUT);
  pinMode(RC_BIT_THROTTLE, INPUT);

  cli();  /*disable*/
  attachInterrupt(digitalPinToInterrupt(RC_BIT_DIRECTION), isrDirectionSignalChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_BIT_THROTTLE), isrThrottleSignalChange, CHANGE);
  sei();  /*enable*/
}

bool rcr_checkRadioStatus()
{
  static bool ret = false;
  static uint32_t prevTs = 0;
  uint32_t ts = millis();
  uint32_t age = ts-prevTs;

  if(age > 1000)
  {
    static uint32_t prevCounters[2] = {0};

    if((pulseCounter[0] == prevCounters[0])
      || (pulseCounter[1] == prevCounters[1]))
    {
      contextInitialize();
      ret = false;
    }
    else
    {
      ret = true;
    }

    prevCounters[0] = pulseCounter[0];
    prevCounters[1] = pulseCounter[1];

    prevTs = ts;
  }

  return ret;
}

