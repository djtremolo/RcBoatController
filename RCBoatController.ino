#include "Common.hpp"

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



Struct              TitleCase
Struct Members      lowerCase

Enum                ETitleCase
Enum Members        ALL_CAPS

Public functions    pfx_TitleCase (pfx = two or three letter module prefix)
Private functions   TitleCase
Trivial variables   i,x,n,f etc...
Local variables     lowerCase
Global variables    g_lowerCase (searchable by g_ prefix)
*/



void setup()
{
  Serial.begin(115200);

  analogReference(INTERNAL);

  pinMode(LED_BUILTIN, OUTPUT);


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
    Serial.print("RC signal changed: ");
    Serial.println(rcStatus ? "OK" : "MISSING");
  }
}


bool timeForNewControlCycle()
{
  return enc_125msElapsed();
}

void loop()
{
  /*only run at 125ms interval*/
  if(timeForNewControlCycle())
  {
    int16_t outR;
    int16_t outL;
    int16_t throttleInPercent;
    int16_t directionInPercent;
    uint32_t encR;
    uint32_t encL;

    /*get input data from radio control and encoder*/
    rcr_getData(throttleInPercent, directionInPercent);
    enc_getData(encR, encL);

    /*prepare output values for motors*/
    ctr_speedControl(outR, outL, throttleInPercent, directionInPercent);

    /*adjust motor output values according to the encoder value*/
    ctr_speedAdjust(outR, outL, encR, encL);

    /*prepare writing values to motor controller*/
    mot_valueSet(0, outR);
    mot_valueSet(1, outL);

    /*activate new motor output values for PWM generation (generation handled by motor controller module) */
    mot_outputUpdate();
  }

  statusMonitor();
}


