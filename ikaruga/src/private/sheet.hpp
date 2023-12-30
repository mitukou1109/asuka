#pragma once

#include <vector>
#include "../common/component/encoder.hpp"
#include "../common/component/motor.hpp"
#include "../common/component/my_timer.hpp"
#include "../common/component/route.hpp"
#include "../common/component/solenoid.hpp"

#undef OK

using std::vector;

class Sheet
{
public:
    
    Sheet();

    void initialize();

    bool getStarted();

    bool startFromTheMiddle(int);

    bool work(MyTimer&);

    void moveBackMechanismToInitialPosition();

    void debug(bool(&)[24]);

    void stopDebug();

    void reset();

private:

    bool pickup(MyTimer&);

    bool hangOut(MyTimer&);

    void checkActionReguraly();

    bool elevateRoger(long);

    bool lowerRoger();

    void stopElevateAndLower();

    void expandSlider();

    void shrinkSlider();

    void gripSheet();

    void releaseSheet();

    void liftHand();

    void dropHand();

    void checkSensorStatus();

private:

    char zone_ = 0;

    char towelIndex_ = 0;

    bool hasCorrected_ = false;

    bool isElevating_ = false;

    bool isLowering_ = false;

    bool isReceived_ = false;

    enum PROCESS_FLAG
    {
        STANDBY,
        START,
        START_FROM_TOWEL,
        APPROACH_MY_FRIEND,
        PICKUP,
        LEAVE_MY_FRIEND,
        APPROACH_POINT_HANG_OUT,
        HANG_OUT,
        LEAVE_POINT_HANG_OUT,
        RETURN,
    } processFlag_ = STANDBY;

    enum ACTION_FLAG
    {
        NONE,
        EXPAND,
        SHRINK,
        PULL,
        PRESS,
        DROP,
        GRIP,
        RELEASE,
    } actionFlag_ = GRIP;

    enum SHEET_COM_CODE
    {
        READY_TO_HANG_OUT = 0,
        GRIP_SHEET = 1,
        GRIPED = 2,
        MOVED = 3,
        PRESS_SHEET = 4,
        HOOK_ON = 5,
        READY_TO_RELEASE = 6,
    };

private:

    const uint8_t
        PIN_LIMIT_ELAVATE_BOTTOM = 30,
        PIN_LIMIT_POLE_GUIDE = 29,
        PIN_LIMIT_CHASSIS_GUIDE_BACK_LEFT = 28,
        PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT = 40,
        PIN_LIMIT_CHASSIS_GUIDE_BACK_RIGHT = 27;

    const long COUNT_2000 = 4900;

    const Vector2
        DIST_MOVE_BACKWARD = {0, 500};

    const double KP_ELEVATE = 0.1;

private:

    Encoder rogerEncorder_;

    Motor rogerMotor_;

    Solenoid sliderSolenoid_, gripperSolenoid_, lifterSolenoid_;

    vector<Vector2> viaPoint_;

    vector<Vector2> pointHangOut_;

    vector<Route> routeSZ2ToMyFriend_, routeSheetToSZ2_;

    vector<vector<vector<Route>>> routeTowelToMyFriend_;
};

extern Sheet sheet;