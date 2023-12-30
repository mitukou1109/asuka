#pragma once

#include "../component/route.hpp"
#include "../component/my_timer.hpp"
#include "../component/motor.hpp"
#include "../component/solenoid.hpp"

class Collection
{
public:

    Collection();

    bool work();

    bool test();

    bool setPositionToCollect(Vector2);

    bool setPositionToDrop(Vector2);

    void initialize();

    void reset();

    void start();

    void debug();

    bool extendArm();

    bool shrinkArm();

    void stopArm();

    void driveConveyor(int);

    void stopConveyor();

    void openMainGate();

    void closeMainGate();

    void openSubGate();

    void closeSubGate();

    void openGate();

    void closeGate();

private:

    static const int
    PIN_LIMIT_CHASSIS_GUIDE_LEFT = A2,
    PIN_LIMIT_CHASSIS_GUIDE_RIGHT = A11,
    PIN_LIMIT_CHASSIS_TOUCH = A6,
    PIN_LIMIT_ARM_EXTEND_RED = A8,
    PIN_LIMIT_ARM_EXTEND_BLUE = A4,
    PIN_LIMIT_ARM_SHRINK = A5;

    enum FLAG
    {
        GO_SZ,
        EXTEND,
        SET_1,
        CLOSE,
        COLLECT,
        WAIT,
        GO,
        SET_2,
        OPEN,
        DROP,
        SET_3
    } flag_ = EXTEND;

    Vector2 collectPoint_, HZPoint_[2];

    Route collectionToHZRoute_[2], SZ1ToHZRoute_[2];

    MyTimer& timer_;

    RelayMotor armMotor_, conveyorMotor_;

    Solenoid mainGateSolenoid_, subGateSolenoid_;
};

extern Collection collection;