#include "Common.hpp"
#include "RcReceiver.hpp"
#include "Controller.hpp"
#include "MotorDriver.hpp"
#include "SpeedEncoder.hpp"
#include "BatteryMonitor.hpp"

/*
btm   BatteryMonitor
ctr   Controller
com   Common
mot   MotorDriver
rcr   RcReceiver
enc   SpeedEncoder
*/



void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);


  Serial.println("cumulativeDiff,encR,encL,outRAfter,outLAfter");


  rcr_initialize();
  mot_initialize();
  enc_initialize();
  ctr_initialize();
  btm_initialize();


}


void statusMonitor()
{
  static bool prevRcStatus = false;
  bool rcStatus = rcr_checkRadioStatus();

  if(rcStatus != prevRcStatus)
  {
#if 0
    Serial.print("RC signal changed: ");
    Serial.println(rcStatus ? "OK" : "MISSING");
#endif
    prevRcStatus = rcStatus;
  }
}


bool timeForNewControlCycle()
{
  return enc_125msElapsed();
}

void loop()
{
  float outR;
  float outL;
  int16_t throttleInPercent;
  int16_t directionInPercent;
  uint32_t encR;
  uint32_t encL;

  static uint32_t cumulativeEncR = 0;
  static uint32_t cumulativeEncL = 0;
  static int32_t cumulativeDiff = 0;

  /*only run at 125ms interval*/
  if(timeForNewControlCycle())
  {
    /*get input data from radio control and encoder*/
    rcr_getData(throttleInPercent, directionInPercent);
    enc_getData(encR, encL);

    cumulativeEncR += encR;
    cumulativeEncL += encL;

#if 1
    if(throttleInPercent > 10)
    {
      throttleInPercent = 10;
      directionInPercent = 0;
    }
    else
    {
      throttleInPercent = directionInPercent = 0; 
    }
#endif



    /*prepare output values for motors*/
    ctr_speedControl(outR, outL, throttleInPercent, directionInPercent);

    /*adjust motor output values according to the encoder value*/
    ctr_speedAdjust(outR, outL, encR, encL);

    /*prepare writing values to motor controller*/
    mot_valueSet(0, outR);
    mot_valueSet(1, outL);

    /*activate new motor output values for PWM generation (generation handled by motor controller module) */
    mot_outputUpdate();


    cumulativeDiff = cumulativeEncR - cumulativeEncL;

  }

#if 1
if(throttleInPercent)
{

  Serial.print(cumulativeDiff);
  Serial.print(",");
  Serial.print((float)encR / 2.5);
  Serial.print(",");
  Serial.print((float)encL / 2.5);
  Serial.print(",");
  Serial.print(outR);
  Serial.print(",");
  Serial.println(outL);
}
#endif



  statusMonitor();
  delay(20);
}


