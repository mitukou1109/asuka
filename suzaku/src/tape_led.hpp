#pragma once

#include <DueTimer.h>

#undef RED
#undef BLUE
#undef GREEN
#undef CYAN
#undef WHITE

#define COLOR_RED TapeLED::COLOR::RED
#define COLOR_BLUE TapeLED::COLOR::BLUE
#define COLOR_GREEN TapeLED::COLOR::GREEN
#define COLOR_YELLOW TapeLED::COLOR::YELLOW
#define COLOR_MAGENTA TapeLED::COLOR::MAGENTA
#define COLOR_CYAN TapeLED::COLOR::CYAN
#define COLOR_WHITE TapeLED::COLOR::WHITE
#define COLOR_NONE TapeLED::COLOR::NONE

class TapeLED
{
    friend void ISR_Blink();
    friend void ISR_SW();

public:

    enum COLOR
    {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        MAGENTA,
        CYAN,
        WHITE,
        NONE,
        NUM_OF_COLORS
    };

    enum STATE
    {
        OFF = LOW,
        ON = HIGH
    };

    TapeLED();

    void initialize();

    bool isSWReleased();

    void set(COLOR);

    void blink(COLOR, COLOR);

    void blink(COLOR);

private:

    const int
    PIN_TAPE_OUT[3] = {37,36,35},
    PIN_SW = 26;

    const STATE COLOR_MAP[NUM_OF_COLORS][3] =
    {
        {ON,OFF,OFF},
        {OFF,ON,OFF},
        {OFF,OFF,ON},
        {ON,ON,OFF},
        {ON,OFF,ON},
        {OFF,ON,ON},
        {ON,ON,ON},
        {OFF,OFF,OFF}
    };

    void turnOn(COLOR);

    void turnOff();

    DueTimer timer_;

    COLOR setColor_[2] = {NONE,NONE};

    bool blinking_ = false;
};

extern TapeLED tape;