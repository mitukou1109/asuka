#pragma once

#include "../component/route.hpp"
#include "../component/my_timer.hpp"
#include "../component/encoder.hpp"
#include "../component/motor.hpp"
#include "../component/solenoid.hpp"

class Sheet
{
public:

    static const int
    COUNT_ROGER_HOOK = 9750,
    COUNT_ROGER_UNFOLD = 10000,
    COUNT_ROGER_RELEASE_SOLO = 8700,
    COUNT_ROGER_RELEASE_DUET = 9000;

    Sheet();

    bool work();

    bool test();

    bool setPositionToCorrect();

    bool setPositionToHook();

    bool setPositionToRelease();

    void initialize();

    void reset();

    void start();

    void debug();

    bool adjustRoger(int);

    bool lowerRoger();

    void stopRoger();

    bool extendSlider();

    bool shrinkSlider();

    void stopSlider();

    void extendHook();

    void shrinkHook();

    void freeHook();

    void gripSheet1();

    void releaseSheet1();

    void freeSheet1();

    void gripSheet2();

    void releaseSheet2();

    void freeSheet2();

    void gripUpperSheet1();

    void releaseUpperSheet1();

    void freeUpperSheet1();

    void gripUpperSheet2();

    void releaseUpperSheet2();

    void freeUpperSheet2();

    void gripUpperSheet();

    void releaseUpperSheet();

    void freeUpperSheet();

    void gripSheet();

    void releaseSheet();

    void freeSheet();

    void extendYamato();

    void freeYamato();

    void releaseDeliverySheet();

    void pinchDeliverySheet();

private:

    static const int
    PIN_LIMIT_CHASSIS_GUIDE_1 = A10,
    PIN_LIMIT_CHASSIS_GUIDE_2 = A6,
    PIN_LIMIT_CHASSIS_TOUCH_X = A11,
    PIN_LIMIT_CHASSIS_TOUCH_Y = 27,
    PIN_LIMIT_CHASSIS_DELIVER = A1,
    PIN_LIMIT_DELIVERY_EXTEND = 25,
    PIN_LIMIT_DELIVERY_SHRINK = 22,
    PIN_LIMIT_DELIVERY_TOUCH = 24,
    PIN_LIMIT_ROGER_BOTTOM = A9;

    enum COM_CODE
    {
        DO_COME = 0,
        DO_GRIP = 1,
        GRIP_OK = 2,
        MOVE_OK = 3,
        SET_OK = 4,
        DO_LOWER = 5,
        DO_RELEASE = 6,
        RETURN_OK = 6
    };

    enum FLAG
    {
        GO_SZ,
        GO,
        GO_X,
        CORRECT,
        WAIT_1,
        DELIVER,
        WAIT_2,
        WAIT_3,
        SET_1,
        WAIT_4,
        SET_2,
        HOOK,
        RELEASE_1,
        LOWER_1,
        RELEASE_U,
        RAISE,
        BACK_1,
        WAIT_5,
        SET_3,
        LOWER_2,
        RELEASE_2,
        BACK_2,
        LOWER_3,
        RETURN_X,
        RETURN
    } flag_ = GO;

    bool isRogerZeroDetected_ = false;

    Vector2 setPoint_[2][5], correctPoint_[2][2];

    Route
        SZToHalfwayRoute_[2],
        HZToCornerRoute_[2], 
        CornerToCornerRoute_, 
        SheetToCornerRoute_[2], 
        CornerToHZRoute_[2];

    MyTimer& timer_;

    Encoder rogerEncoder_;

    Motor rogerMotor_, sliderMotor_;

    Solenoid deliverySolenoid_, gripperU1Solenoid_, gripperU2Solenoid_,  gripper1Solenoid_, gripper2Solenoid_, hookSolenoid_, yamatoSolenoid_;    
};

extern Sheet sheet;