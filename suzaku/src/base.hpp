#pragma once

#include "component/my_timer.hpp"

class Base
{
public:

    Base();

    void work();

    void test();

    void run();

    void initialize();

    void reset();

    void start();

    void exit();

    void setLED(int, bool);

    bool isStarterPressed();

    bool isOnTest();

    MyTimer timer_;

private:

    bool isBusy();

    const int
    PIN_ESCAPE = A7,
    PIN_STARTER_1 = A3,
    PIN_STARTER_2 = 28,
    PIN_LED_ONBOARD[4] = {50,51,52,53};

    bool isAtWork_ = false, isOnTest_ = false, isTshirtDone_ = false;
};

extern Base base;