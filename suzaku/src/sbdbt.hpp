#pragma once

#include <DueTimer.h>

class SBDBT
{
    friend void raiseTransmitFlag();

public:

    SBDBT();

    void initialize();

    void transmit(uint8_t);

    int receive();

    void flush();

private:

    const int
    NO_DATA = 666,
    TRANSMIT_RATE = 100;

    volatile bool transmitFlag_ = true;

    int dataPrev_ = NO_DATA;

    USARTClass& serialBT_;

    DueTimer timer_;
};

extern SBDBT bt;