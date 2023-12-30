#include "tape_led.hpp"
#include <Arduino.h>

void ISR_Blink()
{
    static int blinkIndex = 0;

    tape.turnOn(tape.setColor_[blinkIndex]);
    blinkIndex ^= 1;
}

void ISR_SW()
{
    if(tape.isSWReleased() && !tape.blinking_) tape.turnOn(tape.setColor_[0]);
    else tape.turnOff();
}

TapeLED::TapeLED() :
timer_(3)
{}

void TapeLED::initialize()
{
    for(auto& pin : PIN_TAPE_OUT) pinMode(pin, OUTPUT);
    attachInterrupt(PIN_SW, ISR_SW, CHANGE);
    timer_.attachInterrupt(ISR_Blink).setPeriod(200000);
    turnOff();
}

bool TapeLED::isSWReleased()
{
    return digitalRead(PIN_SW);
}

void TapeLED::set(COLOR color)
{
    if(blinking_)
    {
        timer_.stop();
        setColor_[1] = COLOR::NONE;
        blinking_ = false;
    }
    setColor_[0] = color;
    turnOn(color);
}

void TapeLED::blink(COLOR color1, COLOR color2)
{
    if(!blinking_ || color1!=setColor_[0] || color2!=setColor_[1])
    {
        setColor_[0] = color1;
        setColor_[1] = color2;
        blinking_ = true;
        turnOn(color1);
        timer_.start();
    }
}

void TapeLED::blink(COLOR color)
{
    blink(color, COLOR::NONE);
}

void TapeLED::turnOn(COLOR color)
{
    if(!isSWReleased())
    {
        turnOff();
        return;
    }
    for(int i=0;i<3;i++) digitalWrite(PIN_TAPE_OUT[i], COLOR_MAP[color][i]);
}

void TapeLED::turnOff()
{
    for(auto& pin : PIN_TAPE_OUT) digitalWrite(pin, STATE::OFF);
}

TapeLED tape;