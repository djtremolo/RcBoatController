#include "MotorDriver.hpp"


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
} pwmDataForIsr_t;

typedef struct
{
  bool enable;
  int16_t value;    /*free to be written at any time*/

  /*two pwm data sets. The non-active one is used for preparing for next PWM cycle. 
  After the data is prepared, the activeIndex is changed to point to new values.*/
  int activePwmDataForIsrIndex;
  pwmDataForIsr_t pwmData[2];
} motorContext_t;



pwmDataForIsr_t pwmDataForIsr[2] = {};
motorContext_t motorContext[2];
volatile uint32_t pwmTimerCntr = 0;

void getMotorPwmValuesISR(int idx, pwmDataForIsr_t *pwmData);
void pwmTimerSetup();
void pwmInitialize();
void setupMotorOutputPins();





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


void pwmTimerSetup(uint32_t hz)
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


inline void mot_setPwmPins(const int idx, volatile uint8_t &val0, volatile uint8_t &val1) __attribute__((always_inline));
void mot_setPwmPins(const int idx, volatile uint8_t &val0, volatile uint8_t &val1)
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


void getMotorPwmValuesISR(int idx, pwmDataForIsr_t *pwmData)
{
  /*Will fail if idx > 1*/
  motorContext_t *m = &(motorContext[idx]);
  int activeIdx = m->activePwmDataForIsrIndex;
  pwmDataForIsr_t *activeData = &(m->pwmData[activeIdx]);

  memcpy((void*)pwmData, (const void*)activeData, sizeof(pwmDataForIsr_t));
}


ISR(TIMER2_COMPA_vect)
{
  int m;

  pwmTimerCntr++;

  for(m = 0; m < 2; m++)
  {
    pwmDataForIsr_t *pd = &pwmDataForIsr[m];

    if(pd->resetCycle)
    {
      /*capture new values*/
      getMotorPwmValuesISR(m, pd);

      mot_setPwmPins(m, pd->activeDutyValue[0], pd->activeDutyValue[1]);
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
            mot_setPwmPins(m, pd->passiveDutyValue[0], pd->passiveDutyValue[1]);
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





void mot_outputUpdate()
{
  int idx;

  for(idx = 0; idx < 2; idx++)
  {
    motorContext_t *m = &(motorContext[idx]);

    int16_t mv = m->enable ? m->value : 0;
    motorDrive_t drv = MOTOR_COAST;

    int safePwmSetIndex = m->activePwmDataForIsrIndex == 0 ? 1 : 0;
    pwmDataForIsr_t *safeSet = &(m->pwmData[safePwmSetIndex]);

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


void mot_valueSet(int idx, int16_t valueInPercent)
{
  if(idx < 2)
  {
    motorContext_t *m = &(motorContext[idx]);
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

void mot_outputEnable(int idx, bool enable)
{
  if(idx < 2)
  {
    motorContext_t *m = &(motorContext[idx]);
    m->enable = enable;
  }
}


void pwmInitialize()
{
  memset((void*)&(pwmDataForIsr[0]), 0, sizeof(pwmDataForIsr_t));
  memset((void*)&(pwmDataForIsr[1]), 0, sizeof(pwmDataForIsr_t));

  pwmDataForIsr[0].resetCycle = true;
  pwmDataForIsr[1].resetCycle = true;
}


void mot_initialize()
{
  int i;

  setupMotorOutputPins();

  pwmInitialize();

  for(i=0; i<2; i++)
  {
    motorContext_t* m = &(motorContext[i]);

    memset(m, 0, sizeof(motorContext_t));
    mot_valueSet(i, 0);
    mot_outputEnable(i, true);
  }

  cli();  /*disable*/
  pwmTimerSetup(40000);
  sei();  /*enable*/
}

