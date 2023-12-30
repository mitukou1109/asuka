#include "my_timer.hpp"

volatile bool MyTimer::isISRCalled_[9] = {};

void ISR0()
{
    MyTimer::isISRCalled_[0] = true;
}

void ISR1()
{
    MyTimer::isISRCalled_[1] = true;
}

void ISR2()
{
    MyTimer::isISRCalled_[2] = true;
}

void ISR3()
{
    MyTimer::isISRCalled_[3] = true;
}

void ISR4()
{
    MyTimer::isISRCalled_[4] = true;
}

void ISR5()
{
    MyTimer::isISRCalled_[5] = true;
}

void ISR6()
{
    MyTimer::isISRCalled_[6] = true;
}

void ISR7()
{
    MyTimer::isISRCalled_[7] = true;
}

void ISR8()
{
    MyTimer::isISRCalled_[8] = true;
}

void (*ISR[])() =
{
    ISR0,
    ISR1,
    ISR2,
    ISR3,
    ISR4,
    ISR5,
    ISR6,
    ISR7,
    ISR8,
};

MyTimer::MyTimer(unsigned short id) :
timer_(id),
id_(id)
{
    timer_.attachInterrupt(ISR[id_]);
}

bool MyTimer::wait(unsigned long milliseconds)
{
    if(isRunning_)
    {
        if(isISRCalled_[id_])
        {
            reset();
            return true;
        }
    }
    else
    {
        timer_.setPeriod(milliseconds*1000).start();
        isRunning_ = true;
    }
    
    return false;
}

void MyTimer::reset()
{
    timer_.stop();
    isISRCalled_[id_] = false;
    isRunning_ = false;
}