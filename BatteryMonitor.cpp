#include "BatteryMonitor.hpp"


static int16_t getBatteryLevelInMilliVolts();

static int16_t getBatteryLevelInMilliVolts()
{
  int32_t bvRaw = analogRead(A4);
  constexpr int32_t factorToMicroVolts = 1074 * (8500 / 1060);
  int16_t bvMilliVolts = ((bvRaw * factorToMicroVolts) / 1000);

  return bvMilliVolts;
}

int16_t btm_getBatteryLevelInPercent()
{
  return (getBatteryLevelInMilliVolts() * 100) / 8200;
}

void btm_initialize()
{
  /*set to internal 1.1V reference*/
  analogReference(INTERNAL);
  
  /*read value couple of times to allow adc to settle*/
  for(int i=0; i<10; i++)
  {
    (void)analogRead(A4);
    delay(1);
  }
}

