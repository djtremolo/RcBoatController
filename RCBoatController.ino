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
  bool resetCycle;

  /*the channels A&B are driven as a pair: typically the active duty cycle is either fwd/rev and the rest is braking*/
  uint8_t activeDutyValue[2]; 
  uint8_t passiveDutyValue[2];

  uint16_t activeWidth; /*copied from pre-calculated values: ticks to stay active*/
  uint16_t totalWidth;  /*copied from pre-calculated values: ticks to rest*/

  motorDrive_t drive;
} pwmChannelPair_t;



typedef struct
{
  bool enable;
  int16_t value;    /*free to be written at any time*/
  int16_t pwmValue; /*copied from value at update()*/

  uint8_t activeDutyValue[2]; 
  uint8_t passiveDutyValue[2];

  uint16_t activeWidth; /*pre-calculated for ISR: ticks to stay active*/
  uint16_t totalWidth;  /*pre-calculated for ISR: ticks to rest*/

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



volatile rcContext_t rcContext[2];
motor_t motor[2];
uint32_t pulseCounter[2] = {0};
bool pwmEnable = false;

static pwmChannelPair_t chPair[2] = {0};




void DBGOUT_1_ON()
{
  constexpr uint8_t mask = 1 << 4;
  PORTD |= mask;
}
void DBGOUT_1_OFF()
{
  constexpr uint8_t mask = ~(1 << 4);
  PORTD &= mask;
}

void DBGOUT_2_ON()
{
  constexpr uint8_t mask = 1 << 5;
  PORTD |= mask;
}
void DBGOUT_2_OFF()
{
  constexpr uint8_t mask = ~(1 << 5);
  PORTD &= mask;
}



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
      int16_t mv = m->value;
      motorDrive_t drv = MOTOR_COAST;
      if(mv != 0)
      {
        drv = ((mv < 0) ? MOTOR_REVERSE : MOTOR_FORWARD);
      }

      int16_t pwmAbs = abs(mv);
      uint16_t tw = map(pwmAbs, 0, 100, 100, 20);   /*30kHz ISR -> 1.5kHz at highest throttle, 300Hz at lowest*/
      uint16_t aw = map(pwmAbs, 0, 100, 0, tw);

      switch(drv)
      {
        case MOTOR_FORWARD:
          /*active part: drive*/
          m->activeDutyValue[0] = HIGH;
          m->activeDutyValue[1] = LOW;

          /*rest: brake*/
          m->passiveDutyValue[0] = HIGH;
          m->passiveDutyValue[1] = HIGH;
          break;

        case MOTOR_REVERSE:
          /*active part: drive*/
          m->activeDutyValue[0] = LOW;
          m->activeDutyValue[1] = HIGH;

          /*rest: brake*/
          m->passiveDutyValue[0] = HIGH;
          m->passiveDutyValue[1] = HIGH;
          break;

        case MOTOR_BRAKE:
          /*active part: brake*/
          m->activeDutyValue[0] = HIGH;
          m->activeDutyValue[1] = HIGH;

          /*rest: coast*/
          m->passiveDutyValue[0] = LOW;
          m->passiveDutyValue[1] = LOW;
          break;

        default:
          /*active part: coast*/
          m->activeDutyValue[0] = LOW;
          m->activeDutyValue[1] = LOW;

          /*rest: coast*/
          m->passiveDutyValue[0] = LOW;
          m->passiveDutyValue[1] = LOW;
          break;
      }

//      cli();
      m->drive = drv;
      m->totalWidth = tw;
      m->activeWidth = aw;
      m->pwmValue = pwmAbs;
//      sei();
    }
    else
    {
//      cli();
      m->drive = MOTOR_COAST;
      m->pwmValue = 100;  /*full cycle*/

      /*active part: coast*/
      m->activeDutyValue[0] = LOW;
      m->activeDutyValue[1] = LOW;

      /*rest: coast*/
      m->passiveDutyValue[0] = LOW;
      m->passiveDutyValue[1] = LOW;

//      sei();
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


void getMotorPwmValues(int idx)
{
  motor_t *m = &(motor[idx]);
  pwmChannelPair_t *cp = &(chPair[idx]);

  cp->totalWidth = m->totalWidth;
  cp->activeWidth = m->activeWidth;

  /*active part: drive*/
  cp->activeDutyValue[0] = m->activeDutyValue[0];
  cp->activeDutyValue[1] = m->activeDutyValue[1];

  /*rest: brake*/
  cp->passiveDutyValue[0] = m->passiveDutyValue[0];
  cp->passiveDutyValue[1] = m->passiveDutyValue[1];

  cp->resetCycle = false;
}

inline void setMotorPinsISR(int idx, uint8_t &val0, uint8_t &val1) __attribute__((always_inline));
void setMotorPinsISR(int idx, uint8_t &val0, uint8_t &val1)
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

uint32_t timerCntr=0;

ISR(TIMER2_COMPA_vect)
{
  int m;

  DBGOUT_1_ON();

  timerCntr++;

  for(m = 0; m < 2; m++)
  {
    pwmChannelPair_t *cp = &(chPair[m]);

    if(cp->resetCycle)
    {
      /*capture new values*/
      getMotorPwmValues(m);

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
      else
      {
        cp->resetCycle = true;
      }      
    }
  }


  DBGOUT_1_OFF();

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


void isrRcIncomingPulse(rcContext_t* c, const uint8_t pinMask)
{
  uint32_t ts = micros();
  volatile bool rising = ((PIND & pinMask) == 0 ? false : true);

  int ch = (pinMask==1<<RC_BIT_DIRECTION) ? 0 : 1;


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
      c->pulseWidth = ts - c->prevRisingTs;
    }

    /*remember context information*/
    c->prevFallingTs = ts;
    c->fallingEdgeSeen = true;
  }

  /*allow measurement only when both edge timestamps have been captured and widths are accepted*/
  if(c->risingEdgeSeen 
    && c->fallingEdgeSeen 
  //  && checkRange(c->cycleWidth, 15000, 18000)    //  TODO move out from ISR to getData()
  //  && checkRange(c->pulseWidth, 0, c->cycleWidth)  //  TODO move out from ISR to getData()
    )
  {
    /*everything looks good*/
    pulseCounter[ch]++;

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
  constexpr uint8_t pinMask = 1 << RC_BIT_DIRECTION;
  isrRcIncomingPulse(&(rcContext[0]), pinMask);
  //pulseCounter[0]++;
}

void isrThrottleSignalChange()
{
  constexpr uint8_t pinMask = 1 << RC_BIT_THROTTLE;
  isrRcIncomingPulse(&(rcContext[1]), pinMask);

  //pulseCounter[1]++;
}

bool getData(uint32_t ch, uint32_t *pulseWidth, uint32_t *cycleWidth)
{
  bool ret = false;

  if(ch < 2)
  {
    rcContext_t *c = &(rcContext[ch]);
//ZAIM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!    
//    if(c->signalOk)
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
    //if((pw>=1110) && (pw<=2064))
    {
      int32_t dp = (int32_t)map(pw, 1110, 2064, -100, 100);

      if(abs(dp) < 5) dp = 0;
      else if(dp > 100) dp = 100;
      else if(dp < -100) dp = -100;

      *directionPercent = (int16_t)dp;

      ret = true;
    }
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

DBGOUT_2_ON();

  if(getSteeringDirection(&directionInPercent))
  {
    if(getThrottlePosition(&throttleInPercent))
    {
      int16_t outL = 0;
      int16_t outR = 0;

      if(throttleInPercent != 0)
#if 0
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
#elif 0
      {
        int32_t throttleInPercent100x = throttleInPercent * 100;
        int32_t throttleInPercent100xAbs = abs(throttleInPercent100x);

        
        int32_t steeringStrengthPct = (50000 / (abs(throttleInPercent100x) / 10));
        int32_t factorL, factorR;
        int32_t adjustedFactorL, adjustedFactorR;
        int32_t nonSaturatedL, nonSaturatedR;
        int32_t preSaturationFactor;
        int32_t saturationFactor;
        
        factorL = 100-directionInPercent;
        factorR = 100+directionInPercent;
        
        adjustedFactorL = (factorL * steeringStrengthPct) / 100;   //x10000 -> keep in 100x
        adjustedFactorR = (factorR * steeringStrengthPct) / 100;
        
        nonSaturatedL = throttleInPercent100x + (adjustedFactorL * throttleInPercent);
        nonSaturatedR = throttleInPercent100x + (adjustedFactorR * throttleInPercent);

        int32_t nsAbsL = abs(nonSaturatedL);
        int32_t nsAbsR = abs(nonSaturatedR);
        
        preSaturationFactor = (nsAbsL > nsAbsR ? nonSaturatedL : nonSaturatedR);
        
        saturationFactor = (((nsAbsL < throttleInPercent100xAbs) && (nsAbsR < throttleInPercent100xAbs)) ? 100 : (throttleInPercent100x * 100) / preSaturationFactor);
        
        outL = (int16_t)((nonSaturatedL * saturationFactor) / 10000);
        outR = (int16_t)((nonSaturatedR * saturationFactor) / 10000);
      }
#elif 1
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

#else
      {
        outL = outR = throttleInPercent;
      }
#endif
      *motorL = outR;
      *motorR = outL;


      Serial.print(directionInPercent);
      Serial.print(",");
      Serial.print(throttleInPercent);
      Serial.print(",");
      Serial.print(outL);
      Serial.print(",");
      Serial.print(outR);
      Serial.println(",-100,100");

      ret = true;
    }
  }

DBGOUT_2_OFF();  
  return ret;
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
  OCR2A = 64;//50;// = (16*10^6) / (8000*16) - 1 (must be <256)     // 124 will generate 16kHz  //68 -> 32kHz  //34 -> 64kHz
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);

  /*prescaler*/
  TCCR2B |= (1 << CS21);
  //TCCR2B |= (1 << CS20);

  /*
  CS22  CS21  CS20
  0     0     1       no prescaling
  0     1     0       clk/8
  0     1     1       clk/32
  
  */



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


  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);


  setupMotorOutputPins();

  motorInitialize();

  initializePwm(true);


  cli();  /*disable*/
  setupRadioControlInput();

///////////////////////////////////////////////////////////  
setupPwmTimer();

  sei();  /*enable*/


}



void loop()
{
  int16_t leftMotorValue, rightMotorValue;

#if 1
  if(speedControl(&leftMotorValue, &rightMotorValue))
  {
    //Serial.println("OK");
    motorValueSet(0, leftMotorValue);
    motorValueSet(1, rightMotorValue);
    motorOutputUpdate();
  }
  else
  {
    Serial.println("FAIL");
    /*fallback to idle*/
    motorValueSet(0, 0);
    motorValueSet(1, 0);
    motorOutputUpdate();
  }
  monitorRcReceiverStatus();

#else

  Serial.print(pulseCounter[0] % 200);
  Serial.print(",");
  Serial.print(pulseCounter[1] % 200);
  Serial.print(",");
  Serial.print(rcContext[0].pulseWidth);
  Serial.print(",");
  Serial.print(rcContext[0].cycleWidth);
  Serial.print(",");
  Serial.print(rcContext[1].pulseWidth);
  Serial.print(",");
  Serial.print(rcContext[1].cycleWidth);
  Serial.println("");
#endif


  delay(100);
}


