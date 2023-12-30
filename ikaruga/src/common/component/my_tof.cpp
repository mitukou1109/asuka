#include "my_tof.hpp"
#include <Wire.h>
#include "../i2c_lcd.hpp"

MyToF::MyToF(uint8_t addr, char pinXshout) :
    addr_(addr),
    pinXshout_(pinXshout),
    tof_()
{
    pinMode(pinXshout_, OUTPUT);
    digitalWrite(pinXshout_, LOW);
}

void MyToF::initialize()
{
    Wire.begin();

    pinMode(pinXshout_, INPUT);
    if (tof_.init())
    {
        tof_.setDistanceMode(VL53L1X::Long);
        tof_.setMeasurementTimingBudget(50000);
        tof_.startContinuous(50);
        tof_.setAddress(addr_);
    }
    else
    {
        LCD.print(0x00, "Failed:init ToF");
        LCD.print(0x10, "Cycle The Power");
        while (1);
    }
}

double MyToF::getDistance()
{
   double distRet = 0;
   for (int i = 0; i < 2; i++)
   {
       distRet += static_cast<double>(tof_.read());
   }
   return round(distRet/2);
}