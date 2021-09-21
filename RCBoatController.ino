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


  Serial.println("encDiff, outRBefore,outLBefore, outRAfter,outLAfter, encR,encL");


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
  return enc_measuringCycleElapsed();
}

void loop()
{
  float outR;
  float outL;
  int16_t throttleInPercent;
  int16_t directionInPercent;
  float encRpsR;
  float encRpsL;

  int32_t encDiff = 0;

  /*only run at 125ms interval*/
  if(timeForNewControlCycle())
  {
    /*get input data from radio control and encoder*/
    rcr_getData(throttleInPercent, directionInPercent);
    enc_getData(encRpsR, encRpsL, encDiff);

    //cumulativeEncR += encR;
    //cumulativeEncL += encL;

#if 0
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

#if 1
if(throttleInPercent)
{
  Serial.print(encDiff);
  Serial.print(",");
  
  Serial.print(outR);
  Serial.print(",");
  Serial.print(outL);
  Serial.print(",");
}
#endif



    /*adjust motor output values according to the encoder value*/
    ctr_speedAdjust(outR, outL, encRpsR, encRpsL);

    /*prepare writing values to motor controller*/
    mot_valueSet(0, outR);
    mot_valueSet(1, outL);

    /*activate new motor output values for PWM generation (generation handled by motor controller module) */
    mot_outputUpdate();


#if 1
if(throttleInPercent)
{

//  Serial.print((float)outR*2.5);
  Serial.print(outR);
  Serial.print(",");
  Serial.print(outL);
  Serial.print(",");
  Serial.print(encRpsR/2.5);
  Serial.print(",");
  Serial.print(encRpsL/2.5);
  Serial.print(",");
  Serial.println(",0,100");
}
#endif


  }




  statusMonitor();
  delay(20);
}


