#include <SoftTimer.h>
#include <SoftPwmTask.h>
#include <BlinkTask.h>


SoftPwmTask pwm0(A0);
SoftPwmTask pwm1(A1);
SoftPwmTask pwm2(A2);
SoftPwmTask pwm3(A3);

BlinkTask heartbeat(LED_BUILTIN, 200, 100, 2, 2000);

#define PIN_CH1     2
#define PIN_CH2     3


// -- Define method signature.
void turnOn(Task* me);
void turnOff(Task* me);
void toggle(Task* me);
void cb(Task* me);

// -- taskOn will be launched on every 2 seconds.
Task taskOn(50, turnOn);
// -- taskOff will be launched on every 1111 milliseconds.
Task taskOff(1111, turnOff);

Task taskToggle(1000, toggle);
Task taskCb(10, cb);



typedef struct 
{
  bool signalOk;
  bool risingEdgeSeen;
  bool fallingEdgeSeen;
  bool rising;
  uint32_t ts;
  uint32_t prevRisingTs;
  uint32_t prevFallingTs;
  uint32_t cycleWidth;
  uint32_t pulseWidth;
  uint16_t valueInPercent;
} rcContext_t;

rcContext_t rcContext[2];

void rcContextInitialize(rcContext_t* c)
{
  memset(c, 0, sizeof(rcContext_t));
}

void ch1Pulse();
void ch2Pulse();


void setup() {
  // -- Mark pin 13 as an output pin.
  //pinMode(13, OUTPUT);

  pinMode(PIN_CH1, INPUT);
  pinMode(PIN_CH2, INPUT);

  Serial.begin(115200);

  // -- Register the tasks to the timer manager. Both tasks will start immediately.
  SoftTimer.add(&taskOn);
  SoftTimer.add(&taskOff);
  SoftTimer.add(&taskToggle);
  SoftTimer.add(&taskCb);

  SoftTimer.add(&pwm0);
  SoftTimer.add(&pwm1);
  SoftTimer.add(&pwm2);
  SoftTimer.add(&pwm3);

  heartbeat.start();

  rcContextInitialize(&(rcContext[0]));
  rcContextInitialize(&(rcContext[1]));

  attachInterrupt(digitalPinToInterrupt(PIN_CH1), ch1Pulse, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CH2), ch2Pulse, CHANGE);
}


bool checkRange(uint32_t val, uint32_t minAccepted, uint32_t maxAccepted)
{
  if((val >= minAccepted) && (val <= maxAccepted))
  {
    return true;
  }
  return false;
}

uint32_t pulseCounter = 0;


void incomingPulse(rcContext_t* c, int pin)
{
  uint32_t ts = micros();
  c->rising = digitalRead(pin) == HIGH;

  pulseCounter++;

  if(c->rising)
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

      if(c->cycleWidth != 0)
      {
        /*calculate value in percent*/
        c->valueInPercent = ((100 * c->pulseWidth) / c->cycleWidth);
      }
    }

    /*remember context information*/
    c->prevFallingTs = ts;
    c->fallingEdgeSeen = true;
  }

  /*allow measurement only when both edge timestamps have been captured and widths are accepted*/
  if(c->risingEdgeSeen 
      && c->fallingEdgeSeen 
//      && checkRange(c->cycleWidth, 15000, 18000)
//      && checkRange(c->pulseWidth, 0, c->cycleWidth)
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


/**
 * Turn ON Arduino's LED on pin 13.
 */
void turnOn(Task* me) {
  //digitalWrite(13, HIGH);

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
    Serial.println(pw);
  }
  else
  {
    Serial.println(0);
  }

/*
  Serial.print(". Counter= ");
  Serial.println(pulseCounter);
*/
/*
  Serial.print("D2=");
  Serial.println(digitalRead(2));
*/

}

/**
 * Turn OFF Arduino's LED on pin 13.
 */
void turnOff(Task* me) {
  //digitalWrite(13, LOW);
  //Serial.println("turnOff");
}

void toggle(Task* me) {
  static bool on = false;
  //digitalWrite(13, (on?HIGH:LOW));

  on = !on;

  //Serial.println("toggle");
  
}


void cb(Task* me) {
  //static bool on = false;
  //digitalWrite(13, (on?HIGH:LOW));
  static int a=0;
  static int b=128;
  static int c=64;
  static int d=255-64;

  static int cntr = 0;


  pwm0.analogWrite(a);
  pwm1.analogWrite(b);
  pwm2.analogWrite(c);
  pwm3.analogWrite(d);


  if(cntr == 0)
  {
    a = (a+1) % 256;
    b = (b+1) % 256;
    c = (c+1) % 256;
    d = (d+1) % 256;
  }  

  cntr = (cntr+1) % 50;

  //Serial.print("cb=");
  //Serial.println(a);
  
}
