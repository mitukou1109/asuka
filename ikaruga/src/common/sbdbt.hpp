#pragma once

#include <Arduino.h>
#include <DueTimer.h>

enum COM_CODE
{
    PAUSE = 5,
    RESUME = 6,
};

class SBDBT
{
    friend void raiseTransmitFlag();

public:

    SBDBT();

    void initialize();

    bool isConnected();

    void write(uint8_t data);

    int read();

    void flush();

private:

    bool isConnected_ = false;

    int dataPrev_ = 0;

    volatile bool transmitFlag_ = true;

private:

    const uint8_t ACK = COM_CODE::RESUME;

    const int
        NO_DATA = 666,
        TRANSMIT_RATE = 100;

private:

    UARTClass& serialBT_;

    DueTimer transmitTimer_;
};

extern SBDBT bt;