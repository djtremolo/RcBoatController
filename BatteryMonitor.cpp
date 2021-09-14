#include "BatteryMonitor.hpp"


int16_t getBatteryLevelInMilliVolts();
int16_t getBatteryLevelInPercents();




int16_t getBatteryLevelInMilliVolts()
{
  int32_t bvRaw = analogRead(A4);
  constexpr int32_t factorToMicroVolts = 1074 * (8500 / 1060);
  int16_t bvMilliVolts = ((bvRaw * factorToMicroVolts) / 1000);

  return bvMilliVolts;
}

int16_t getBatteryLevelInPercents()
{
  return (getBatteryLevelInMilliVolts() * 100) / 8000;
}

