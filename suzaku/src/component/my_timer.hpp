#pragma once

#include <DueTimer.h>

class MyTimer
{
    friend void ISR0();
    friend void ISR1();
    friend void ISR2();
    friend void ISR3();
    friend void ISR4();
    friend void ISR5();
    friend void ISR6();
    friend void ISR7();
    friend void ISR8();

public:

    MyTimer(unsigned short);

    bool wait(unsigned long);

    void reset();

private:

    DueTimer timer_;

    static volatile bool isISRCalled_[];

    bool isRunning_ = false;

    unsigned short id_;
};