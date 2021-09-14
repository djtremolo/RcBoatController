#include <stdint.h>
#include "Arduino.h"
#include "PinChangeInterrupt.h"

#define RC_INPUT_REGISTER           PIND
#define RC_DIRECTION_REGISTER       DDRD
#define RC_OUTPUT_REGISTER          PORTD

#define RC_BIT_DIRECTION            2
#define RC_BIT_THROTTLE             3


#define SS_INPUT_REGISTER           PIND
#define SS_DIRECTION_REGISTER       DDRD
#define SS_OUTPUT_REGISTER          PORTD

#define SS_BIT_SENSOR1              4
#define SS_BIT_SENSOR2              5


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
} rcContext_t;

typedef enum
{
  MOTOR_COAST,
  MOTOR_FORWARD,
  MOTOR_REVERSE,
  MOTOR_BRAKE
} motorDrive_t;

typedef volatile struct
{
  bool resetCycle;

  uint8_t activeDutyValue[2];   
  uint8_t passiveDutyValue[2];

  uint16_t activeWidth; /*pre-calculated for ISR: ticks to stay active*/
  uint16_t totalWidth;  /*pre-calculated for ISR: ticks to rest*/
} PwmDataForIsr_t;

typedef struct
{
  bool enable;
  int16_t value;    /*free to be written at any time*/

  /*two pwm data sets. The non-active one is used for preparing for next PWM cycle. 
  After the data is prepared, the activeIndex is changed to point to new values.*/
  int activePwmDataForIsrIndex;
  PwmDataForIsr_t pwmData[2];
} motor_t;



void rcContextInitialize(int ch);
void rcContextInitialize(void);
void motorInitialize();
void motorOutputUpdate();
void motorValueSet(int idx, int16_t valueInPercent);
void motorEnable(int idx, bool enable);
void monitorRcReceiverStatus();
void initializePwm();
void getMotorPwmValuesISR(int idx, PwmDataForIsr_t *pwmData);
bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted);
void isrRcIncomingPulse(rcContext_t* c, int pin, const uint32_t ts);
void isrDirectionSignalChange();
void isrThrottleSignalChange();
bool getData(uint32_t ch, uint32_t *pulseWidth);
int16_t getSteeringDirection();
int16_t getThrottlePosition();
void speedControl();
int16_t getBatteryLevelInMilliVolts();
int16_t getBatteryLevelInPercents();
void setupRadioControlInput();
void setupPwmTimer();
void setupMotorOutputPins();

void isrSpeedSensor0();
void isrSpeedSensor1();
uint32_t speedSensorRawValueGet(const int ch);
uint32_t speedSensorRpmAbsGet(const int ch);

void speedAdjust(int16_t& outR, int16_t& outL);


PwmDataForIsr_t pwmDataForIsr[2] = {};
rcContext_t rcContext[2];
volatile uint32_t speedSensorTicks[2];
volatile uint32_t speedSensorEighthsOfRevolution[2];
volatile uint32_t speedSensorSeqNo;   /*incremented each time the RPS values have been updated. Used for synchronizing the speedAdjust.*/
motor_t motor[2];
uint32_t pulseCounter[2] = {0};
bool pwmEnable = false;

volatile uint32_t sgTimerCntr = 0;
volatile uint32_t pwmTimerCntr = 0;


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

void rcContextInitialize(int ch)
{
  if(ch < 2)
  {
    rcContext_t* c = &(rcContext[ch]);
    memset((void*)c, 0, sizeof(rcContext_t));
  }
}

void rcContextInitialize(void)
{
  rcContextInitialize(0);
  rcContextInitialize(1);
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

    int16_t mv = m->enable ? m->value : 0;
    motorDrive_t drv = MOTOR_COAST;

    int safePwmSetIndex = m->activePwmDataForIsrIndex == 0 ? 1 : 0;
    PwmDataForIsr_t *safeSet = &(m->pwmData[safePwmSetIndex]);

    if(mv != 0)
    {
      drv = ((mv < 0) ? MOTOR_REVERSE : MOTOR_FORWARD);
    }

    int16_t pwmAbs = abs(mv);
    uint16_t tw = 100;//map(pwmAbs, 0, 100, 100, 40);   /*40kHz ISR -> 1000Hz at highest throttle, 400Hz at lowest*/
    uint16_t aw = map(pwmAbs, 0, 100, 0, tw);

    switch(drv)
    {
      case MOTOR_FORWARD:
        /*active part: drive*/
        safeSet->activeDutyValue[0] = HIGH;
        safeSet->activeDutyValue[1] = LOW;

        /*rest: brake*/
        safeSet->passiveDutyValue[0] = HIGH;
        safeSet->passiveDutyValue[1] = HIGH;
        break;

      case MOTOR_REVERSE:
        /*active part: drive*/
        safeSet->activeDutyValue[0] = LOW;
        safeSet->activeDutyValue[1] = HIGH;

        /*rest: brake*/
        safeSet->passiveDutyValue[0] = HIGH;
        safeSet->passiveDutyValue[1] = HIGH;
        break;

      case MOTOR_BRAKE:
        /*active part: brake*/
        safeSet->activeDutyValue[0] = HIGH;
        safeSet->activeDutyValue[1] = HIGH;

        /*rest: coast*/
        safeSet->passiveDutyValue[0] = LOW;
        safeSet->passiveDutyValue[1] = LOW;
        break;

      default:
        /*active part: coast*/
        safeSet->activeDutyValue[0] = LOW;
        safeSet->activeDutyValue[1] = LOW;

        /*rest: coast*/
        safeSet->passiveDutyValue[0] = LOW;
        safeSet->passiveDutyValue[1] = LOW;
        break;
    }

    safeSet->totalWidth = tw;
    safeSet->activeWidth = aw;
    safeSet->resetCycle = false;

    /*values have now been prepared, activate the new set as used*/
    m->activePwmDataForIsrIndex = safePwmSetIndex;
  }
}


void motorValueSet(int idx, int16_t valueInPercent)
{
  if(idx < 2)
  {
    motor_t *m = &(motor[idx]);
    constexpr int16_t posMax = 100;
    constexpr int16_t negMax = -posMax;

    if(valueInPercent > posMax)
    {
      valueInPercent = posMax;
    }
    else if(valueInPercent < negMax)
    {
      valueInPercent = negMax;
    }

    m->value = valueInPercent;
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

void initializePwm()
{
  memset((void*)&(pwmDataForIsr[0]), 0, sizeof(PwmDataForIsr_t));
  memset((void*)&(pwmDataForIsr[1]), 0, sizeof(PwmDataForIsr_t));

  pwmDataForIsr[0].resetCycle = true;
  pwmDataForIsr[1].resetCycle = true;
}


void getMotorPwmValuesISR(int idx, PwmDataForIsr_t *pwmData)
{
  /*Will fail if idx > 1*/
  motor_t *m = &(motor[idx]);
  int activeIdx = m->activePwmDataForIsrIndex;
  PwmDataForIsr_t *activeData = &(m->pwmData[activeIdx]);

  memcpy((void*)pwmData, (const void*)activeData, sizeof(PwmDataForIsr_t));
}

inline void setMotorPinsISR(const int idx, volatile uint8_t &val0, volatile uint8_t &val1) __attribute__((always_inline));
void setMotorPinsISR(const int idx, volatile uint8_t &val0, volatile uint8_t &val1)
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
  int m;

  pwmTimerCntr++;

  for(m = 0; m < 2; m++)
  {
    PwmDataForIsr_t *pd = &pwmDataForIsr[m];

    if(pd->resetCycle)
    {
      /*capture new values*/
      getMotorPwmValuesISR(m, pd);

      setMotorPinsISR(m, pd->activeDutyValue[0], pd->activeDutyValue[1]);
    }
    else
    {
      if(pd->totalWidth > 0)
      {
        pd->totalWidth--;

        /*check if it changed from one to zero*/
        if(pd->totalWidth == 0)
        {
          pd->resetCycle = true;
        }
        if(pd->activeWidth > 0)
        {
          pd->activeWidth--;

          /*check if it changed from one to zero*/
          if(pd->activeWidth == 0)
          {
            /*clear output*/
            setMotorPinsISR(m, pd->passiveDutyValue[0], pd->passiveDutyValue[1]);
          }
        }
      }
      else
      {
        pd->resetCycle = true;
      }      
    }
  }
}


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


bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted)
{
  if((val >= minAccepted) && (val <= maxAccepted))
  {
    return true;
  }

  return false;
}

uint32_t rcCntr = 0;


void isrRcIncomingPulse(const int ch, const uint8_t pinMask, const uint32_t ts)
{
  bool rising = ((PIND & pinMask) == 0 ? false : true);
  rcContext_t* c = &(rcContext[ch]);

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

uint32_t getFilteredPulseWidth(rcContext_t *c)
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
    rcContext_t *c = &(rcContext[ch]);
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



int16_t getBatteryLevelInMilliVolts()
{
  int32_t bvRaw = analogRead(A4);
  constexpr int32_t factorToMicroVolts = 1074 * (8500 / 1060);
  int16_t bvMilliVolts = ((bvRaw * factorToMicroVolts) / 1000);

  return bvMilliVolts;
}

int16_t getBatteryLevelInPercents()
{
  return (getBatteryLevelInMilliVolts() * 100) / 8000;
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


void setupPwmTimer(uint32_t hz)
{
  uint32_t prescalers[] = {1, 8, 32};
  uint32_t chosenPrescaler = 0;
  uint32_t val = 0;
  int i;

  TCCR2A = 0;
  TCCR2B = 0;

  // initialize counter to zero
  TCNT2  = 0;

  // turn on CTC mode
  TCCR2A |= (1 << WGM21);

  /*
  CS22  CS21  CS20
  0     0     1       no prescaling
  0     1     0       clk/8
  0     1     1       clk/32  
  */

  /*
    hz = 16M / ((val+1)*prescaler)
    hz/16M = 1/ ((val+1)*prescaler)
    16M/hz = (val+1)*prescaler
    (16M/hz) / prescaler = val+1
  */

  for(i = 0; i<3; i++)
  {
    val =  ((16000000 / hz) / prescalers[i]) -1;

    if(val > 0 && val < 256)
    {
      //found!
      chosenPrescaler = prescalers[i];
      break;
    }
  }

  switch(chosenPrescaler)  
  {
    case 1:
      TCCR2B |= (1 << CS20);
      break;
    case 8:
      TCCR2B |= (1 << CS21);
      break;
    case 32:
      TCCR2B |= (1 << CS20);
      TCCR2B |= (1 << CS21);
      break;
    default:
      return;
  }

  OCR2A = val;

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

  pinMode(LED_BUILTIN, OUTPUT);

  setupMotorOutputPins();

  motorInitialize();

  initializePwm();


  cli();  /*disable*/
  setupSpeedSensor();
  setupspeedAdjustTimer();
  setupPwmTimer(40000);
  setupRadioControlInput();
  sei();  /*enable*/
}

void loop()
{
  speedControl();
  monitorRcReceiverStatus();

  delay(10);
}


