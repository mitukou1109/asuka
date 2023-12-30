#pragma once

#include "../component/route.hpp"
#include "../component/my_timer.hpp"
#include "../component/encoder.hpp"
#include "../component/motor.hpp"
#include "../component/solenoid.hpp"

class Tshirt
{
public:

    static const int
    COUNT_WING_HOOK = 16500,
    COUNT_WING_RELEASE = 2500;

    Tshirt();

    bool work();

    bool test();

    bool setPositionToHook();

    void initialize();

    void reset();

    void start();

    void debug();

    bool adjustWing(int);

    bool lowerWing();

    void stopWing();

    void gripHanger();

    void releaseHanger();

    void freeHanger(); 

private:

    static const int
    PIN_LIMIT_POLE_TOUCH = A0,
    PIN_LIMIT_WING_LOWER = 23;

    enum FLAG
    {
        GO_SZ,
        GO,
        SET,
        LOWER,
        RELEASE_1,
        RELEASE_2,
        BACK,
        RETURN
    } flag_ = GO;

    Vector2 setPoint_[2][2];

    Route SZToTshirtRoute_[2][2], HZToTshirtRoute_[2][2], TshirtToHZRoute_[2][2];

    bool isWingZeroDetected_ = false;

    MyTimer& timer_;

    Encoder wingEncoder_;

    Motor wingMotor_;

    Solenoid hangerSolenoid_;
};

extern Tshirt tshirt;