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

typedef struct
{
  bool enable;
  int16_t value;
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

void ch1Pulse();
void ch2Pulse();


uint32_t pulseCounter = 0;
uint32_t timerCounter = 0;


uint8_t pwmOutputValues[4] = {0};
#define M1_FWD  0
#define M1_REV  1
#define M2_FWD  2
#define M2_REV  3

int pwmOutputPins[4] = {A0, A1, A2, A3};

bool pwmEnable = false;


void motorOutputUpdateByIndex(int idx)
{
  if(idx < 2)
  {
    motor_t *m = &(motor[idx]);
    const int fwdPin = (idx == 0 ? M1_FWD : M2_FWD);
    const int revPin = (idx == 0 ? M1_REV : M2_REV);

    if(m->enable)
    {
      if(m->value > 0)
      {
        /*clear backward output*/
        pwmOutputValues[revPin] = 0;
        pwmOutputValues[fwdPin] = m->value;
      }
      else if(m->value < 0)
      {
        /*clear backward output*/
        pwmOutputValues[fwdPin] = 0;
        pwmOutputValues[revPin] = -1 * m->value;
      }
    }
  }
}

void motorOutputUpdate()
{
  motorOutputUpdateByIndex(0);
  motorOutputUpdateByIndex(1);
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


ISR(TIMER2_COMPA_vect)
{
  /*8kHz interval*/
  static uint32_t tickCounter = 0;
  static uint8_t values[4];
  int i;

  timerCounter++;

  if(pwmEnable)
  {
    if(tickCounter == 0)
    {
      /*capture new values*/
      for(i=0; i<4; i++)
      {
        values[i] = pwmOutputValues[i];
      }

      for(i=0; i<4; i++)
      {
        digitalWrite(pwmOutputPins[i], ((values[i] != 0) ? HIGH : LOW));
      }
    }
    else
    {
      for(i=0; i<4; i++)
      {
        if(values[i] > 0)
        {
          values[i]--;

          /*check if it changed from one to zero*/
          if(values[i] == 0)
          {
            /*clear output*/
            digitalWrite(pwmOutputPins[i], LOW);
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

  pulseCounter++;

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

void ch1Pulse()
{
  incomingPulse(&(rcContext[0]), PIN_CH1);
}
void ch2Pulse()
{
  incomingPulse(&(rcContext[1]), PIN_CH2);
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

  Serial.println(val);

  val = (val+1) % 200;

//  motorValueSet(0, val-100);
  if(getSteeringDirection(&dir))
  {
    motorValueSet(0, dir);
  }

  motorOutputUpdate();
  
  delay(100);

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


  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  sei();  /*enable*/

  motorValueSet(0, 0);
  motorValueSet(1, 0);

  motorEnable(0, true);
  motorEnable(1, true);


  pwmEnable = true;
}
