#include "sbdbt.hpp"
#include <Arduino.h>

void raiseTransmitFlag()
{
    bt.transmitFlag_ = true;
    bt.transmitTimer_.stop();
}

SBDBT::SBDBT() :
serialBT_(Serial2),
transmitTimer_(2)
{}

void SBDBT::initialize()
{
    serialBT_.begin(9600);
    transmitTimer_.attachInterrupt(raiseTransmitFlag).setPeriod(TRANSMIT_RATE*1000);
}

bool SBDBT::isConnected()
{
    write(COM_CODE::RESUME);
    if(not isConnected_)
    {
        if(read() == COM_CODE::RESUME)
        {
            isConnected_ = true;
        }
    }
    return isConnected_;
}

void SBDBT::write(uint8_t data)
{
    if(transmitFlag_ or data != dataPrev_)
    {
        serialBT_.write(data);
        transmitTimer_.start();
        dataPrev_ = data;
        transmitFlag_ = false;
    }
}

int SBDBT::read()
{
    if(serialBT_.available() > 0)
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
    while (serialBT_.available() > 0)
    {
        serialBT_.read();
    }
}

SBDBT bt;