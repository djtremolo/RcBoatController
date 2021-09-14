#include "Common.hpp"




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


