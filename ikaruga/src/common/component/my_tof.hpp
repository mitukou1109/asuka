#pragma once

#include <VL53L1X.h>

class MyToF
{
public:

    MyToF(uint8_t, char);

    void initialize();

    double getDistance();

private:
    
    uint8_t addr_;

    char pinXshout_;

    VL53L1X tof_;
};