#include "Arduino.h"
#include <stdint.h>
#define PIN_CH1     2   /*note: this flexible configuraion is NOT used in ext0 ISR*/
#define PIN_CH2     3   /*note: this flexible configuraion is NOT used in ext1 ISR*/

#define ISR_DIVIDER   32   // MUST be power of 2!
constexpr int16_t pwmRangeMin = -1 * (256 / ISR_DIVIDER);
constexpr int16_t pwmRangeMax = 256 / ISR_DIVIDER;



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
//PORTC: bit0 = A0 etc

bool pwmEnable = false;


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

int16_t motorPwmValueGet(int idx)
{
  if(idx < 2)
  {
    motor_t *m = &(motor[idx]);
    return m->pwmValue;
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

uint16_t tmpDriveRestValue = HIGH;
bool tmpBoostEnabled = false;

bool applyBoost(uint8_t *width, int *cntr)
{
  bool ret = false;

  if(tmpBoostEnabled)
  {
    if(*cntr <= 5)
    {
      if(*width > 0)
      {
        ret = true;
        *width = 128;
      }
      else if(*width < 0)
      {
        ret = true;
        *width = -128;
      }
    }

    *cntr = (*cntr +1) % 100;
  }

  return ret;
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
      cp->totalWidth = 100; //map(abs(m->pwmValue), 10, 90, 2000, 100);
      cp->activeWidth = map(abs(m->pwmValue), 10, 90, 0, cp->totalWidth);
//    cp->totalWidth = map(abs(m->pwmValue), 0, 100, 2000, 100);
//    cp->activeWidth = map(abs(m->pwmValue), 0, 100, 0, cp->totalWidth);
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


static pwmChannelPair_t chPair[2] = {0};

void initializePwm(bool enable)
{
  pwmEnable = false;
  memset(&(chPair[0]), 0, sizeof(pwmChannelPair_t));
  memset(&(chPair[1]), 0, sizeof(pwmChannelPair_t));

  chPair[0].resetCycle = true;
  chPair[1].resetCycle = true;

  pwmEnable = enable;  
}

ISR(TIMER2_COMPA_vect)
{
  static bool cycleDone[2] = {0};
  int m;

  digitalWrite(4, HIGH);

  timerCounter++;

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

  digitalWrite(4, LOW);
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

  //digitalWrite(5, HIGH);

  bool rising = PORTD & (0x1 << pin);//digitalRead(pin) == HIGH;

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


  //digitalWrite(5, LOW);
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
  static int16_t val = 0;
  int16_t dir=0;
  static bool up = true;
  static int16_t factor = 1;
  static bool rControl = false;
  int16_t av;

  if(rControl)
  {
    getSteeringDirection(&av);
  }
  else
  {
    av = factor * val;
  }
  
  motorValueSet(0, av);
  motorValueSet(1, av);

  motorOutputUpdate();

  delay(100);

  if (Serial.available() > 0) 
  {
    char ch = Serial.read();
    if(ch>='0' && ch <='9')
    {
      val = (int16_t)(ch-'0')*10;
    }
    else if(ch=='-')
    {
      factor = -1;
    }
    else if(ch=='+')
    {
      factor = 1;
    }
    else if(ch=='r' || ch == 'R')
    {
      tmpDriveRestValue = (tmpDriveRestValue == HIGH ? LOW : HIGH);
    }
    else if(ch=='b' || ch == 'B')
    {
      tmpBoostEnabled = !tmpBoostEnabled;
    }
    else if(ch=='c' || ch == 'C')
    {
      rControl = !rControl;
    }



    Serial.print("swFreq=");
    Serial.print(64000 / pwmRangeMax);
    Serial.print(", rest=");
    Serial.print(tmpDriveRestValue == HIGH ? "Brake" : "Coast");
    Serial.print(", rControl=");
    Serial.println(rControl ? "True" : "False");

    Serial.print("val=");
    Serial.print(av);
    Serial.print(", m0Raw=");
    Serial.println(motorPwmValueGet(0));

  }


  //monitorRcReceiverStatus();
}





void setup()
{
  int i;
  // -- Mark pin 13 as an output pin.
  //pinMode(13, OUTPUT);

  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);
  pinMode(13, OUTPUT);


  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);


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
  // set compare match register for 64khz increments (with no prescaler)
//  OCR2A = 249;// = (16*10^6) / (8000*16) - 1 (must be <256)     // 124 will generate 16kHz
  OCR2A = 50;// = (16*10^6) / (8000*16) - 1 (must be <256)     // 124 will generate 16kHz  //68 -> 32kHz  //34 -> 64kHz
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);

  /*prescaler*/
  TCCR2B |= (1 << CS21);    //CS21 for division by 8
//  TCCR2B |= (1 << CS20);    //CS20 to disable prescaler


  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);


  /*the ISR will be called at 16kHz interval -> 128 pwm steps -> 8ms per cycle*/

  sei();  /*enable*/

  motorValueSet(0, 0);
  motorValueSet(1, 0);

  motorEnable(0, true);
  motorEnable(1, true);

  initializePwm(true);
}
