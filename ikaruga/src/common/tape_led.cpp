#include "tape_led.hpp"
#include <Arduino.h>

void ISRBlink()
{
    if(not tape.blinking_ and not tape.blinkingMultiColor_)
    {
        return;
    }
    
    if (tape.blinking_)
    {
        if (tape.state_ == TapeLED::STATE::ON)
        {
            tape.set(tape.blinkColor_);
            tape.state_ = TapeLED::STATE::OFF;
        }
        else if (tape.state_ == TapeLED::STATE::OFF)
        {
            tape.turnOff();
            tape.state_ = TapeLED::STATE::ON;
        }
    }
    else if (tape.blinkingMultiColor_)
    {
        if (tape.state_ == TapeLED::STATE::ON)
        {
            tape.set(tape.blinkColor_);
            tape.state_ = TapeLED::STATE::OFF;
        }
        else if (tape.state_ == TapeLED::STATE::OFF)
        {
            tape.set(tape.blinkColor1_);
            tape.state_ = TapeLED::STATE::ON;
        }
    }
}

TapeLED::TapeLED(char pinRed, char pinBlue, char pinGreen, char pinYellow) :
    blinkTimer_(3)
{
    pin_[0] = pinRed;
    pin_[1] = pinBlue;
    pin_[2] = pinGreen;
    pin_[3] = pinYellow;

    for(auto& pin : pin_)
    {
        pinMode(pin, OUTPUT);
    }
    pinMode(PIN_SW, INPUT);
    blinkTimer_.attachInterrupt(ISRBlink);
}

void TapeLED::set(COLOR color)
{
    stopBlink();

    if(color == COLOR::NONE or not digitalRead(PIN_SW))
    {
        turnOff();
        return;
    }
    for(int i = 0; i < 4; i++)
    {
        if(color & (1 << i))
        {
            digitalWrite(pin_[i], LOW);
        }
        else
        {
            digitalWrite(pin_[i], HIGH);
        }
    }
}

void TapeLED::turnOff()
{
    for(auto& pin : pin_)
    {
        digitalWrite(pin, HIGH);
    }
}

void TapeLED::blink(COLOR color, unsigned int milliseconds)
{
    if(not blinking_)
    {
        blinkColor_ = color;
        blinkTimer_.setPeriod(milliseconds*1000).start();
        blinkingMultiColor_ = false;
        blinking_ = true;
    }
}

void TapeLED::blinkMultiColor(COLOR color, COLOR color1, unsigned int milliseconds)
{
    if (not blinkingMultiColor_)
    {
        blinkColor_ = color;
        blinkColor1_ = color1;
        blinkTimer_.setPeriod(milliseconds*1000).start();
        blinking_ = false;
        blinkingMultiColor_ = true;
    }
}

void TapeLED::stopBlink()
{
    blinkTimer_.stop();
    blinkColor_ = COLOR::NONE;
    state_ = STATE::ON;
    blinking_ = false;
    blinkingMultiColor_ = false;
}

TapeLED tape(22,23,24,25);