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


typedef volatile struct 
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


typedef volatile struct
{
  uint32_t prevGlitchTs;
  uint32_t prevRealChangeTs;
  bool trackingLevel;
  uint32_t detectedPulseInUs;  
} speedSensorContext_t;



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
void isrRcIncomingPulse(rcContext_t* c, int pin);
void isrDirectionSignalChange();
void isrThrottleSignalChange();
bool getData(uint32_t ch, uint32_t *pulseWidth, uint32_t *cycleWidth);
int16_t getSteeringDirection();
int16_t getThrottlePosition();
float calculateSteeringStrength(int16_t throttle);
void speedControl();
int16_t getBatteryLevel();
void setupRadioControlInput();
void setupPwmTimer();
void setupMotorOutputPins();

void isrSpeedSensorCommon(const int ch, const bool risingEdge);
void isrSpeedSensor0();
void isrSpeedSensor1();
uint32_t speedSensorRawValueGet(const int ch);
uint32_t speedSensorRpmAbsGet(const int ch);


PwmDataForIsr_t pwmDataForIsr[2] = {};
rcContext_t rcContext[2];
speedSensorContext_t sensorContext[2];
motor_t motor[2];
uint32_t pulseCounter[2] = {0};
bool pwmEnable = false;


void sensorContextInitialize(int ch)
{
  if(ch < 2)
  {
    speedSensorContext_t* c = &(sensorContext[ch]);
    memset((void*)c, 0, sizeof(speedSensorContext_t));
  }
}

void sensorContextInitialize(void)
{
  sensorContextInitialize(0);
  sensorContextInitialize(1);
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


const uint32_t speedSensorMinimumPeriodInUs = 400; /*TODO: change this to be a function of requested speed*/

void isrSpeedSensorCommon(const int ch)
{
  speedSensorContext_t* c = &(sensorContext[ch]);
  uint32_t ts = micros();
  const uint32_t pulseTime = ts - c->prevRealChangeTs;

  c->detectedPulseInUs = pulseTime;
  c->prevRealChangeTs = ts;
}


void isrSpeedSensor0()
{
  isrSpeedSensorCommon(0);
}

void isrSpeedSensor1()
{
  isrSpeedSensorCommon(1);
}

uint32_t speedSensorRawValueGet(const int ch)
{
  uint32_t val = 0;
  speedSensorContext_t* c = &(sensorContext[ch]);
  uint32_t pulseWidth = c->detectedPulseInUs;

  /*clear after reading*/
  c->detectedPulseInUs = 0;

  val = pulseWidth;

  return val;
}

uint32_t speedSensorRpmAbsGet(const int ch)
{
  constexpr uint32_t usInMilliSecond = 1000;
  constexpr uint32_t usInSecond = usInMilliSecond * 1000;
  constexpr uint32_t usInMinute = usInSecond * 60;
  constexpr uint32_t cyclesPerRevolution = 2;
  constexpr uint32_t dividend = usInMinute / cyclesPerRevolution;

  uint32_t rpm = 0;
  uint32_t pulseInUs = speedSensorRawValueGet(ch);

  if(pulseInUs == 0)
  {
    rpm = 0;
  }
  else if(pulseInUs < 1000)
  {
    rpm = 30000;
  }
  else if(pulseInUs > 500000)
  {
    rpm = 60;
  }
  else
  {
    rpm = dividend / pulseInUs;
  }

  return rpm;
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
    uint16_t tw = map(pwmAbs, 0, 100, 100, 20);   /*30kHz ISR -> 1.5kHz at highest throttle, 300Hz at lowest*/
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

uint32_t timerCntr=0;

ISR(TIMER2_COMPA_vect)
{
  int m;

  timerCntr++;

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

bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted)
{
  if((val >= minAccepted) && (val <= maxAccepted))
  {
    return true;
  }

  return false;
}

uint32_t rcCntr = 0;


void isrRcIncomingPulse(const int ch, const uint8_t pinMask)
{
  uint32_t ts = micros();
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
      c->pulseWidth = ts - c->prevRisingTs;
    }

    /*remember context information*/
    c->prevFallingTs = ts;
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
  constexpr uint8_t pinMask = 1 << RC_BIT_DIRECTION;
  isrRcIncomingPulse(0, pinMask);
}

void isrThrottleSignalChange()
{
  constexpr uint8_t pinMask = 1 << RC_BIT_THROTTLE;
  isrRcIncomingPulse(1, pinMask);
}

bool getData(uint32_t ch, uint32_t *pulseWidth, uint32_t *cycleWidth)
{
  bool ret = false;

  if(ch < 2)
  {
    rcContext_t *c = &(rcContext[ch]);

    if(c->signalOk
        && checkRange(c->cycleWidth, 15000, 18000)
        && checkRange(c->pulseWidth, 0, c->cycleWidth)
    )
    {
      /*TODO: the reading should be atomic*/
      *pulseWidth = c->pulseWidth;
      *cycleWidth = c->cycleWidth;
      ret = true;
    }
  }

  return ret;
}

int16_t getSteeringDirection()
{
  int16_t steeringDirection = 0;
  uint32_t pw, cw;

  if(getData(0, &pw, &cw))
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

    throttlePosition = (int16_t)tp;
  }

  return throttlePosition;
}

float calculateSteeringStrength(int16_t throttle)
{
  float pct;

  pct = (5.0 / (abs(throttle) / 10.0));

  return pct;
}

static uint32_t prevRpm0, prevRpm1;


void speedControl()
{
  int16_t throttleInPercent = getThrottlePosition();
  int16_t directionInPercent = getSteeringDirection();
  int16_t outL = 0;
  int16_t outR = 0;


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


  motorValueSet(0, outR);
  motorValueSet(1, outL);

  motorOutputUpdate();


  return;
}


void speedGovernor()
{
  uint32_t zaimRpm0 = speedSensorRpmAbsGet(0);
  uint32_t zaimRpm1 = speedSensorRpmAbsGet(1);

  //if((zaimRpm0 != prevRpm0) || (zaimRpm1 != prevRpm1))
  {
    Serial.println(zaimRpm1);
    //Serial.println("RPM");
  }

  prevRpm0 = zaimRpm0;
  prevRpm1 = zaimRpm1;
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

  setupPwmTimer(30000);
  setupRadioControlInput();
  sei();  /*enable*/
}

void loop()
{
  speedControl();
////////////////////////////////// ZAIM  monitorRcReceiverStatus();

  speedGovernor();

  //while(1) 
  delay(100);
}


