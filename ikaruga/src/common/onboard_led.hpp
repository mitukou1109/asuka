#pragma once

class OnboardLED
{
public:

    const int
        PIN_LED_ONBOARD[4] = {50, 51, 52, 53};

    OnboardLED();

    void set(int, bool);

    void turnOff();

    void checkPin(int, int);
};

extern OnboardLED LED;