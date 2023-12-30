#pragma once

#include <DueTimer.h>
#undef RED
#undef BLUE
#undef GREEN
#undef WHITE
#undef ON
#undef OFF

#define COLOR_RED     TapeLED::COLOR::RED
#define COLOR_BLUE    TapeLED::COLOR::BLUE
#define COLOR_GREEN   TapeLED::COLOR::GREEN
#define COLOR_YELLOW  TapeLED::COLOR::YELLOW
#define COLOR_MAGENTA TapeLED::COLOR::MAGENTA
#define COLOR_CYAN    TapeLED::COLOR::CYAN
#define COLOR_PINK    TapeLED::COLOR::PINK
#define COLOR_WHITE   TapeLED::COLOR::WHITE
#define COLOR_NONE    TapeLED::COLOR::NONE


class TapeLED
{
    friend void ISRBlink();

public:

    TapeLED(char, char, char, char);

    enum COLOR
    {
        NONE    = 0b0000,
        RED     = 0b0001,
        BLUE    = 0b0010,
        GREEN   = 0b0100,
        YELLOW  = 0b1000,
        MAGENTA = 0b0011,
        CYAN    = 0b0110,
        PINK    = 0b1001,
        WHITE   = 0b1111,
    };

    enum STATE
    {
        OFF,
        ON
    };

    void set(COLOR);

    void turnOff();

    void blink(COLOR, unsigned int);

    void blinkMultiColor(COLOR, COLOR, unsigned int);

    void stopBlink();

private:
    
    const uint8_t PIN_SW = A1;

private:

    DueTimer blinkTimer_;

    char pin_[4];

    volatile bool blinkFlag_ = false;

    volatile STATE state_ = ON;

    COLOR blinkColor_ = NONE;

    COLOR blinkColor1_ = NONE;

    bool blinkingMultiColor_ = false;

    bool blinking_ = false;
};

extern TapeLED tape;