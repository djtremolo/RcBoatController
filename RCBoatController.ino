#include "Arduino.h"
#include <stdint.h>
#define PIN_CH1     2
#define PIN_CH2     3

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
  uint8_t width;  /*0...127*/

  /*the channels A&B are driven as a pair: typically the active duty cycle is either fwd/rev and the rest is braking*/
  uint8_t activeDutyValue[2]; 
  uint8_t passiveDutyValue[2];
} pwmChannelPair_t;



typedef struct
{
  bool enable;
  int16_t value;    /*free to be written at any time*/
  int16_t pwmValue; /*copied from value at update()*/
  motorDrive_t drive;
} motor_t;

rcContext_t rcContext[2];
motor_t motor[2];

void rcContextInitialize(rcContext_t* c)
{
  memset(c, 0, sizeof(rcContext_t));
}

void motorInitialize(motor_t* m)
{
  memset(m, 0, sizeof(motor_t));
}

void getMotorPwmValues(pwmChannelPair_t* chPair);


void ch1Pulse();
void ch2Pulse();


uint32_t pulseCounter[2] = {0};
uint32_t timerCounter = 0;

#define M1_FWD  0
#define M1_REV  1
#define M2_FWD  2
#define M2_REV  3

int pwmOutputPins[4] = {A0, A1, A2, A3};

bool pwmEnable = false;


void motorOutputUpdate()
{
  int idx;

  for(idx = 0; idx < 2; idx++)
  {
    motor_t *m = &(motor[idx]);

    if(m->enable)
    {
      m->drive = ((m->value < 0) ? MOTOR_REVERSE : MOTOR_FORWARD);
      m->pwmValue = m->value;
    }
    else
    {
      m->drive = MOTOR_COAST;
      m->pwmValue = 128;  /*full cycle*/
    }
  }
}


void motorValueSet(int idx, int16_t valueInPercent)
{
  if(idx < 2)
  {
    motor_t *m = &(motor[idx]);
    m->value = map(valueInPercent, -100, 100, -128, 128);
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
      rcContextInitialize(&rcContext[0]);
      pwmEnable = false;
    }

    if((pulseCounter[1] == prevCounters[1]))
    {
      Serial.println("ERROR: Ch2 receiver pulse not detected");
      rcContextInitialize(&rcContext[1]);
      pwmEnable = false;
    }

    prevCounters[0] = pulseCounter[0];
    prevCounters[1] = pulseCounter[1];

    prevTs = ts;
  }
}


void getMotorPwmValues(pwmChannelPair_t* chPair)
{
  int i;
  for(i = 0; i < 2; i++)
  {
    motor_t *m = &(motor[i]);
    pwmChannelPair_t *cp = &(chPair[i]);

    cp->width = m->pwmValue;
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
        cp->passiveDutyValue[0] = HIGH;
        cp->passiveDutyValue[1] = HIGH;
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

ISR(TIMER2_COMPA_vect)
{
  static uint32_t tickCounter = 0;
  static pwmChannelPair_t chPair[2];
  int i;

  timerCounter++;

  /////////////////////if(pwmEnable)
  if(1)
  {
    if(tickCounter == 0)
    {
      /*capture new values*/
      getMotorPwmValues(chPair);

      for(i=0; i<2; i++)
      {
        const int pinIdx = i*2;
        pwmChannelPair_t *cp = &(chPair[i]);

        digitalWrite(pwmOutputPins[pinIdx], cp->activeDutyValue[0]);
        digitalWrite(pwmOutputPins[pinIdx+1], cp->activeDutyValue[1]);
      }
    }
    else
    {
      for(i=0; i<2; i++)
      {
        pwmChannelPair_t *cp = &(chPair[i]);

        if(cp->width > 0)
        {
          cp->width--;

          /*check if it changed from one to zero*/
          if(cp->width == 0)
          {
            /*clear output*/
            const int pinIdx = i*2;

            digitalWrite(pwmOutputPins[pinIdx], cp->passiveDutyValue[0]);
            digitalWrite(pwmOutputPins[pinIdx+1], cp->passiveDutyValue[1]);
          }
        }
      }
    }
  }
  else
  {
    /*not enabled, clear all*/
    for(i=0; i<4; i++)
    {
      digitalWrite(pwmOutputPins[i], LOW);
    }
  }

  tickCounter = (tickCounter+1) % 128;
}



bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted)
{
  if((val >= minAccepted) && (val <= maxAccepted))
  {
    return true;
  }
  return false;
}


void incomingPulse(rcContext_t* c, int pin)
{
  uint32_t ts = micros();
  bool rising = digitalRead(pin) == HIGH;

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
    pwmEnable = true;
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

void ch1Pulse()
{
  incomingPulse(&(rcContext[0]), PIN_CH1);
  pulseCounter[0]++;
}
void ch2Pulse()
{
  incomingPulse(&(rcContext[1]), PIN_CH2);
  pulseCounter[1]++;
}

bool getData(uint32_t ch, uint32_t *pulseWidth, uint32_t *cycleWidth)
{
  bool ret = false;

  if(ch < 2)
  {
    rcContext_t *c = &(rcContext[ch]);
    if(c->signalOk)
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
    int32_t dp = (int32_t)map(pw, 1116, 2064, -100, 100);

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
    int32_t tp = (int32_t)map(pw, 1116, 2064, -100, 100);

    if(abs(tp) < 5) tp = 0;
    else if(tp > 100) tp = 100;
    else if(tp < -100) tp = -100;

    *throttlePosition = (int16_t)tp;

    ret = true;
  }

  return ret;
}


bool speedControl()
{
  int16_t throttle;
  int16_t direction;

  if(getSteeringDirection(&direction))
  {
    if(getThrottlePosition(&throttle))
    {
      float leftMotor; 
      float rightMotor;
      float divider = ((float)abs(throttle)) / 10.0;
      if(divider < 1.0) divider = 1.0;
      float steeringStrength = 2.0 / divider; /*value will be in range of 200% ... 20% depending on throttle*/

      float primaryMotorScale = 1.0;
      float anotherMotorScale = (((float)abs(direction)) * steeringStrength) / 100.0;

      if(throttle > 0)
      {
        /*forward*/
      }
    }
  }


}



void loop()
{
#if 0
  uint32_t pw, cw;
  int16_t dir;

  if(getSteeringDirection(&dir))
  {
    Serial.print(dir);
  }
  else
  {
    Serial.print(0);
  }

  Serial.print("\t");

  if(getData(0, &pw, &cw))
  {
    Serial.print(pw);
    Serial.print("\t");
    Serial.println(cw);
  }
  else
  {
    Serial.println("0\t0");
  }
#endif

  static int16_t val = 0;
  int16_t dir=0;


  val = (val+1) % 11;

  #if 0
//  motorValueSet(0, val-100);
  if(getSteeringDirection(&dir))
  {
    motorValueSet(0, dir);
  }
  #endif

  Serial.println(val*10);
  motorValueSet(0, val*10);
  motorValueSet(1, -1*(val*10));

  motorOutputUpdate();
  
  delay(1000);

  monitorRcReceiverStatus();
}





void setup()
{
  int i;
  // -- Mark pin 13 as an output pin.
  //pinMode(13, OUTPUT);

  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  pinMode(13, OUTPUT);

  for(i=0; i<4; i++)
  {    
    pinMode(pwmOutputPins[i], OUTPUT);
    digitalWrite(pwmOutputPins[i], LOW);
  }


  Serial.begin(115200);

  rcContextInitialize(&(rcContext[0]));
  rcContextInitialize(&(rcContext[1]));

  motorInitialize(&(motor[0]));
  motorInitialize(&(motor[1]));

  cli();  /*disable*/

  attachInterrupt(digitalPinToInterrupt(PIN_CH1), ch1Pulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH2), ch2Pulse, CHANGE);


  //set timer2 interrupt at 16kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 16khz increments (with no prescaler)
  OCR2A = 124;// = (16*10^6) / (8000*16) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  //Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


  /*the ISR will be called at 16kHz interval -> 128 pwm steps -> 8ms per cycle*/

  sei();  /*enable*/

  motorValueSet(0, 0);
  motorValueSet(1, 0);

  motorEnable(0, true);
  motorEnable(1, true);


  pwmEnable = true;
}
