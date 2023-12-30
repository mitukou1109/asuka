#include "onboard_led.hpp"
#include <Arduino.h>

OnboardLED::OnboardLED()
{
    for(auto& pin : PIN_LED_ONBOARD)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void OnboardLED::set(int index, bool level)
{
    digitalWrite(PIN_LED_ONBOARD[index], level);
}

void OnboardLED::turnOff()
{
    for(int i=0;i<4;i++)
    {
        set(i, LOW);
    }
}

void OnboardLED::checkPin(int index, int pin)
{
    set(index, digitalRead(pin));
}

OnboardLED LED;