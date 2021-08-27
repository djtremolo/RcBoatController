#include "Arduino.h"
#include <stdint.h>


#define RC_INPUT_REGISTER           PIND
#define RC_DIRECTION_REGISTER       DDRD
#define RC_OUTPUT_REGISTER          PORTD

#define RC_BIT_DIRECTION            2
#define RC_BIT_THROTTLE             3



typedef struct 
{
  bool signalOk;
  bool risingEdgeSeen;
  bool fallingEdgeSeen;
  uint32_t prevRisingTs;
  uint32_t prevFallingTs;
  uint32_t cycleWidth;
  uint32_t pulseWidth;
} rcContext_t;

typedef enum
{
  MOTOR_COAST,
  MOTOR_FORWARD,
  MOTOR_REVERSE,
  MOTOR_BRAKE
} motorDrive_t;


typedef struct 
{
  uint16_t activeWidth; /*ticks to stay active*/
  uint16_t totalWidth;  /*ticks to rest*/

  /*the channels A&B are driven as a pair: typically the active duty cycle is either fwd/rev and the rest is braking*/
  uint8_t activeDutyValue[2]; 
  uint8_t passiveDutyValue[2];

  bool resetCycle;
} pwmChannelPair_t;



typedef struct
{
  bool enable;
  int16_t value;    /*free to be written at any time*/
  int16_t pwmValue; /*copied from value at update()*/
  motorDrive_t drive;
} motor_t;


void rcContextInitialize(rcContext_t* c);
void motorInitialize();
void motorOutputUpdate();
void motorValueSet(int idx, int16_t valueInPercent);
void motorEnable(int idx, bool enable);
void monitorRcReceiverStatus();
void initializePwm(bool enable);
void getMotorPwmValues(pwmChannelPair_t* chPair);
bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted);
void isrRcIncomingPulse(rcContext_t* c, int pin);
void isrDirectionSignalChange();
void isrThrottleSignalChange();
bool getData(uint32_t ch, uint32_t *pulseWidth, uint32_t *cycleWidth);
bool getSteeringDirection(int16_t *directionPercent);
bool getThrottlePosition(int16_t *throttlePosition);
float calculateSteeringStrength(int16_t throttle);
bool speedControl(int16_t *motorL, int16_t *motorR);
int16_t getBatteryLevel();
void setupRadioControlInput();
void setupPwmTimer();
void setupMotorOutputPins();



rcContext_t rcContext[2];
motor_t motor[2];
uint32_t pulseCounter[2] = {0};
bool pwmEnable = false;

static pwmChannelPair_t chPair[2] = {0};



void rcContextInitialize(rcContext_t* c)
{
  memset(c, 0, sizeof(rcContext_t));
}

void motorInitialize()
{
  int i;
  for(i=0; i<2; i++)
  {
    motor_t* m = &(motor[i]);

    memset(m, 0, sizeof(motor_t));
    motorValueSet(i, 0);
    motorEnable(i, true);
  }
}

void motorOutputUpdate()
{
  int idx;

  for(idx = 0; idx < 2; idx++)
  {
    motor_t *m = &(motor[idx]);

    if(m->enable)
    {
      if(m->value == 0)
      {
        m->drive = MOTOR_COAST;
      }
      else
      {
        m->drive = ((m->value < 0) ? MOTOR_REVERSE : MOTOR_FORWARD);
      }

      m->pwmValue = abs(m->value);
    }
    else
    {
      m->drive = MOTOR_COAST;
      m->pwmValue = 100;  /*full cycle*/
    }
  }
}


void motorValueSet(int idx, int16_t valueInPercent)
{
  if(idx < 2)
  {
    motor_t *m = &(motor[idx]);
    m->value = valueInPercent; //map(valueInPercent, -100, 100, pwmRangeMin, pwmRangeMax);
  }
}

void motorEnable(int idx, bool enable)
{
  if(idx < 2)
  {
    motor_t *m = &(motor[idx]);
    m->enable = enable;
  }
}


void monitorRcReceiverStatus()
{
  static uint32_t prevTs = 0;
  static uint32_t prevRcCounter = 0;
  uint32_t ts = millis();
  uint32_t age = ts-prevTs;

  if(age > 50) /*normal cycle is 50ms*/
  {
    static uint32_t prevCounters[2] = {0};

    if((pulseCounter[0] == prevCounters[0]))
    {
      Serial.println("ERROR: Ch1 receiver pulse not detected");
//      rcContextInitialize(&rcContext[0]);
      pwmEnable = false;
    }

    if((pulseCounter[1] == prevCounters[1]))
    {
      Serial.println("ERROR: Ch2 receiver pulse not detected");
//      rcContextInitialize(&rcContext[1]);
      pwmEnable = false;
    }

    prevCounters[0] = pulseCounter[0];
    prevCounters[1] = pulseCounter[1];

    prevTs = ts;
  }
}

uint16_t tmpDriveRestValue = HIGH;
bool tmpBoostEnabled = false;

void initializePwm(bool enable)
{
  pwmEnable = false;
  memset(&(chPair[0]), 0, sizeof(pwmChannelPair_t));
  memset(&(chPair[1]), 0, sizeof(pwmChannelPair_t));

  chPair[0].resetCycle = true;
  chPair[1].resetCycle = true;

  pwmEnable = enable;  
}


void getMotorPwmValues(pwmChannelPair_t* chPair)
{
  int i;
  for(i = 0; i < 2; i++)
  {
    motor_t *m = &(motor[i]);
    pwmChannelPair_t *cp = &(chPair[i]);

    if(m->pwmValue != 0)
    {
      int16_t pwmAbs = abs(m->pwmValue);
      
      cp->totalWidth = map(pwmAbs, 0, 100, 100, 20);
      cp->activeWidth = map(pwmAbs, 0, 100, 0, cp->totalWidth);
    }
    else
    {
      cp->totalWidth = 100;
      cp->activeWidth = 0;
    }

    cp->resetCycle = false;

    switch(m->drive)
    {
      case MOTOR_FORWARD:

        /*active part: drive*/
        cp->activeDutyValue[0] = HIGH;
        cp->activeDutyValue[1] = LOW;

        /*rest: brake*/
        cp->passiveDutyValue[0] = HIGH;
        cp->passiveDutyValue[1] = HIGH;
        break;

      case MOTOR_REVERSE:
        /*active part: drive*/
        cp->activeDutyValue[0] = LOW;
        cp->activeDutyValue[1] = HIGH;

        /*rest: brake*/
        cp->passiveDutyValue[0] = HIGH;
        cp->passiveDutyValue[1] = HIGH;
        break;

      case MOTOR_BRAKE:
        /*active part: brake*/
        cp->activeDutyValue[0] = HIGH;
        cp->activeDutyValue[1] = HIGH;

        /*rest: coast*/
        cp->passiveDutyValue[0] = LOW;
        cp->passiveDutyValue[1] = LOW;
        break;

      default:
        /*active part: coast*/
        cp->activeDutyValue[0] = LOW;
        cp->activeDutyValue[1] = LOW;

        /*rest: coast*/
        cp->passiveDutyValue[0] = LOW;
        cp->passiveDutyValue[1] = LOW;
        break;
    }
  }
}

inline void setMotorPinsISR(int idx, uint8_t val0, uint8_t val1) __attribute__((always_inline));
void setMotorPinsISR(int idx, uint8_t val0, uint8_t val1)
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


ISR(TIMER2_COMPA_vect)
{
  static bool cycleDone[2] = {0};
  int m;

  if(pwmEnable)
  {
    for(m = 0; m < 2; m++)
    {
      pwmChannelPair_t *cp = &(chPair[m]);

      if(cp->resetCycle)
      {
        /*capture new values*/
        getMotorPwmValues(cp);

        setMotorPinsISR(m, cp->activeDutyValue[0], cp->activeDutyValue[1]);
      }
      else
      {
        if(cp->totalWidth > 0)
        {
          cp->totalWidth--;

          /*check if it changed from one to zero*/
          if(cp->totalWidth == 0)
          {
            cp->resetCycle = true;
          }

          if(cp->activeWidth > 0)
          {
            cp->activeWidth--;

            /*check if it changed from one to zero*/
            if(cp->activeWidth == 0)
            {
              /*clear output*/
              setMotorPinsISR(m, cp->passiveDutyValue[0], cp->passiveDutyValue[1]);
            }
          }
        }
      }
    }
  }
  else
  {
    /*not enabled, clear all*/
    setMotorPinsISR(0, LOW, LOW);
    setMotorPinsISR(1, LOW, LOW);
  }
}

bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted)
{
  if((val >= minAccepted) && (val <= maxAccepted))
  {
    return true;
  }
  return false;
}

uint32_t rcCntr = 0;


void isrRcIncomingPulse(rcContext_t* c, int pin)
{
  uint32_t ts = micros();
  bool rising = PIND & (0x1 << pin);

  rcCntr++;

  if(rising)
  {
    if(c->risingEdgeSeen && c->fallingEdgeSeen)
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
    if(c->risingEdgeSeen && c->fallingEdgeSeen)
    {
      /*calculate pulse width*/
      c->pulseWidth = ts - c->prevRisingTs;
    }

    /*remember context information*/
    c->prevFallingTs = ts;
    c->fallingEdgeSeen = true;
  }

  /*allow measurement only when both edge timestamps have been captured and widths are accepted*/
  if(c->risingEdgeSeen 
    && c->fallingEdgeSeen 
    && checkRange(c->cycleWidth, 15000, 18000)
    && checkRange(c->pulseWidth, 0, c->cycleWidth)
    )
  {
    /*everything looks good*/
    c->signalOk = true;
  }
  else
  {
    if(c->signalOk)
    {
      /*the receiver still thinks signal is OK but it is not -> reset state*/
      rcContextInitialize(c);
    }
  }
}

void isrDirectionSignalChange()
{
  isrRcIncomingPulse(&(rcContext[0]), RC_BIT_DIRECTION);
  pulseCounter[0]++;
}

void isrThrottleSignalChange()
{
  isrRcIncomingPulse(&(rcContext[1]), RC_BIT_THROTTLE);
  pulseCounter[1]++;
}

bool getData(uint32_t ch, uint32_t *pulseWidth, uint32_t *cycleWidth)
{
  bool ret = false;

  if(ch < 2)
  {
    rcContext_t *c = &(rcContext[ch]);
//ZAIM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    if(c->signalOk)
    {
      /*TODO: the reading should be atomic*/
      *pulseWidth = c->pulseWidth;
      *cycleWidth = c->cycleWidth;
      ret = true;
    }
  }

  return ret;
}

bool getSteeringDirection(int16_t *directionPercent)
{
  bool ret = false;
  uint32_t pw, cw;

  if(getData(0, &pw, &cw))
  {
    int32_t dp = (int32_t)map(pw, 1110, 2064, -100, 100);

    if(abs(dp) < 5) dp = 0;
    else if(dp > 100) dp = 100;
    else if(dp < -100) dp = -100;

    *directionPercent = (int16_t)dp;

    ret = true;
  }

  return ret;
}

bool getThrottlePosition(int16_t *throttlePosition)
{
  bool ret = false;
  uint32_t pw, cw;

  if(getData(1, &pw, &cw))
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

    *throttlePosition = (int16_t)tp;

    ret = true;
  }

  return ret;
}

float calculateSteeringStrength(int16_t throttle)
{
  float pct;

  pct = (5.0 / (abs(throttle) / 10.0));

  return pct;
}


bool speedControl(int16_t *motorL, int16_t *motorR)
{
  bool ret = false;
  int16_t throttleInPercent;
  int16_t directionInPercent;

  if(getSteeringDirection(&directionInPercent))
  {
    if(getThrottlePosition(&throttleInPercent))
    {
      int16_t outL = 0;
      int16_t outR = 0;

      if(throttleInPercent != 0)
      {
        float throttle = throttleInPercent / 100.0;
        float steeringStrength = calculateSteeringStrength(throttleInPercent);

        float factorL, factorR;
        float adjustedFactorL, adjustedFactorR;
        float nonSaturatedL, nonSaturatedR;
        float preSaturationFactor;
        float saturationFactor;

        factorL = (float)((100-directionInPercent) / 100.0);
        factorR = (float)((100+directionInPercent) / 100.0);

        adjustedFactorL = (float)(factorL * steeringStrength);
        adjustedFactorR = (float)(factorR * steeringStrength);

        nonSaturatedL = (float)(throttle + (adjustedFactorL * throttle));
        nonSaturatedR = (float)(throttle + (adjustedFactorR * throttle));

        preSaturationFactor = (abs(nonSaturatedL) > abs(nonSaturatedR) ? nonSaturatedL : nonSaturatedR);

        saturationFactor = (((abs(nonSaturatedL) < throttle) && abs(nonSaturatedR) < throttle) ? 1.0 : throttle / preSaturationFactor);

        outL = (int16_t)((nonSaturatedL * saturationFactor) * 100.0);
        outR = (int16_t)((nonSaturatedR * saturationFactor) * 100.0);
      }

      *motorL = outL;
      *motorR = outR;

      ret = true;
    }
  }
}

int16_t getBatteryLevel()
{
  int32_t bvRaw = analogRead(A4);
  constexpr int32_t factorToMicroVolts = 1074 * (8500 / 1060);
  int16_t bvMilliVolts = ((bvRaw * factorToMicroVolts) / 1000);

  return bvMilliVolts;
}

void setupRadioControlInput()
{
  rcContextInitialize(&(rcContext[0]));
  rcContextInitialize(&(rcContext[1]));

  constexpr uint8_t mask = (1 << RC_BIT_DIRECTION) | (1 << RC_BIT_THROTTLE);

  RC_DIRECTION_REGISTER = RC_DIRECTION_REGISTER & (~mask);  //force chosen bits to zero = input

  /*Assign interrupt service routines to both ports. TODO: change to non-arduino later, as the isr uses raw IO*/
  attachInterrupt(digitalPinToInterrupt(RC_BIT_DIRECTION), isrDirectionSignalChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_BIT_THROTTLE), isrThrottleSignalChange, CHANGE);
}


void setupPwmTimer()
{
  //set timer2 interrupt at 16kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 64khz increments (with no prescaler)
//  OCR2A = 249;// = (16*10^6) / (8000*16) - 1 (must be <256)     // 124 will generate 16kHz
  OCR2A = 249;//50;// = (16*10^6) / (8000*16) - 1 (must be <256)     // 124 will generate 16kHz  //68 -> 32kHz  //34 -> 64kHz
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);

  /*prescaler*/
  TCCR2B |= (1 << CS21);    //CS21 for division by 8
//  TCCR2B |= (1 << CS20);    //CS20 to disable prescaler


  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
}

void setupMotorOutputPins()
{
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);

  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
}

void setup()
{
  Serial.begin(115200);

  analogReference(INTERNAL);

  pinMode(13, OUTPUT);

  setupMotorOutputPins();

  motorInitialize();

  cli();  /*disable*/
  setupRadioControlInput();
  setupPwmTimer();
  sei();  /*enable*/


  initializePwm(true);
}

void loop()
{
  int16_t leftMotorValue, rightMotorValue;

  if(speedControl(&leftMotorValue, &rightMotorValue))
  {
    motorValueSet(0, leftMotorValue);
    motorValueSet(1, rightMotorValue);
    motorOutputUpdate();
  }
  else
  {
    /*fallback to idle*/
    motorValueSet(0, 0);
    motorValueSet(1, 0);
    motorOutputUpdate();
  }

  monitorRcReceiverStatus();

  delay(1);
}


