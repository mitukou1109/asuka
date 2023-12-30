#include "sbdbt.hpp"
#include <Arduino.h>

void raiseTransmitFlag()
{
    bt.transmitFlag_ = true;
}

SBDBT::SBDBT() :
serialBT_(Serial2),
timer_(2)
{}

void SBDBT::initialize()
{
    serialBT_.begin(9600);
    timer_.attachInterrupt(raiseTransmitFlag).setPeriod(TRANSMIT_RATE*1000);
}

void SBDBT::transmit(uint8_t data)
{
    if(transmitFlag_ || data!=dataPrev_)
    {
        serialBT_.write(data);
        dataPrev_ = data;
        transmitFlag_ = false;
        timer_.start();
    }
}

int SBDBT::receive()
{
    if(serialBT_.available()>0)
    {
        return serialBT_.read();
    }
    else
    {
        return NO_DATA;
    }
}

void SBDBT::flush()
{
    while(serialBT_.available()>0)
    {
        serialBT_.read();
    }
}

SBDBT bt;