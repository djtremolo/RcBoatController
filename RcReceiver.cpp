#include "RcReceiver.hpp"



constexpr int rcFilterBufferSize = 6;


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


void rcContextInitialize(int ch);
void rcContextInitialize(void);

void setupRadioControlInput();

void isrRcIncomingPulse(radioControlContext_t* c, int pin, const uint32_t ts);
void isrDirectionSignalChange();
void isrThrottleSignalChange();
bool getData(uint32_t ch, uint32_t *pulseWidth);

void monitorRcReceiverStatus();









void rcContextInitialize(int ch)
{
  if(ch < 2)
  {
    radioControlContext_t* c = &(rcContext[ch]);
    memset((void*)c, 0, sizeof(radioControlContext_t));
  }
}

void rcContextInitialize(void)
{
  rcContextInitialize(0);
  rcContextInitialize(1);
}





void setupRadioControlInput()
{
  rcContextInitialize();

#if 0
  constexpr uint8_t mask = (1 << RC_BIT_DIRECTION) | (1 << RC_BIT_THROTTLE);

  RC_DIRECTION_REGISTER = RC_DIRECTION_REGISTER & (~mask);  //force chosen bits to zero = input
#endif

  pinMode(RC_BIT_DIRECTION, INPUT);
  pinMode(RC_BIT_THROTTLE, INPUT);

  /*Assign interrupt service routines to both ports. TODO: change to non-arduino later, as the isr uses raw IO*/
  attachInterrupt(digitalPinToInterrupt(RC_BIT_DIRECTION), isrDirectionSignalChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_BIT_THROTTLE), isrThrottleSignalChange, CHANGE);
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

  for(i = 0; i < rcFilterBufferSize; i++)
  {
    sum += c->pulseWidthArray[i];
  }

  return round((float)sum / rcFilterBufferSize);
}

bool getData(uint32_t ch, uint32_t *pulseWidth)
{
  bool ret = false;

  if(ch < 2)
  {
    radioControlContext_t *c = &(rcContext[ch]);
    uint32_t pwAvg;

    pwAvg = getFilteredPulseWidth(c);

    if(c->signalOk
        && checkRange(c->cycleWidth, 15000, 18000)
        && checkRange(pwAvg, 0, c->cycleWidth)
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

  if(getData(0, &pw))
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

  if(getData(1, &pw))
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




void monitorRcReceiverStatus()
{
  static uint32_t prevTs = 0;
  uint32_t ts = millis();
  uint32_t age = ts-prevTs;

  if(age > 1000)
  {
    static uint32_t prevCounters[2] = {0};

    if((pulseCounter[0] == prevCounters[0]))
    {
      Serial.println("ERROR: Direction pulse not detected");
      rcContextInitialize(0);
    }

    if((pulseCounter[1] == prevCounters[1]))
    {
      Serial.println("ERROR: Throttle pulse not detected");
      rcContextInitialize(1);
    }

    prevCounters[0] = pulseCounter[0];
    prevCounters[1] = pulseCounter[1];

    prevTs = ts;
  }
}

