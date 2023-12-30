#pragma once

#include <vector>
#include "../common/component/encoder.hpp"
#include "../common/component/motor.hpp"
#include "../common/component/my_timer.hpp"
#include "../common/component/route.hpp"
#include "../common/component/solenoid.hpp"
#include "../common/component/vector2.hpp"

using std::vector;

class Towel
{
public:

    Towel();
    
    void initialize();

    bool getStarted();

    bool work(MyTimer& timer);

    void moveBackMechanismToInitialPosition();

    bool elevateElevator(long);

    bool elevateElevatorFor1500();

    bool LowerElevator(long);

    bool resetElevator();

    void stopElevateAndLower();

    void debug(bool(&status)[24]);

    void stopDebug();

    void reset();

    bool initialPos_ = true;

private:

    bool reloadTowel(MyTimer& timer);

    bool hangOutTowel(MyTimer& timer);

    void checkActionRegularly();

    bool correctOdometryWithLeftLimit(double correctX, MyTimer& correctionTimer);

    bool correctOdometryWithRightLimit(double correctX, MyTimer& correctionTimer);

    bool extendArms();

    bool shrinkArms();

    bool extendArmsByDefault();

    bool shrinkArmsByDefault();

    void stopExpandAndShrink();

    void gripTowel();
    
    void releaseTowel();

    void swingUp();

    void swingDown();

    void liftMagazine();

    void dropMagazine();
    
    void checkSensorStatus();

private:
    
    char zone_ = 0;

    char round_ = 0;

    bool workSheet_ = false;

    char towelIndex_ = 0;

    long countTop_ = 0;

    vector<vector<bool>> completeTowel_;

    bool isFirstTowel_ = true;

    bool hasSetheight_ = false;

    bool hasDecided_ = false;

    bool isAtPoint_ = false;

    bool hasPassedViaPoint_ = false;

    bool isMovingForTowel2_ = false;

    bool hasApproached_ = false;

    bool hasCorrectedYAxis_ = false;

    bool hasCorrected_ = false;

    bool hasReloaded_ = false;

    bool hasLowered_ = false;

    bool towel0_ = false;

    char swingStatus_ = 0;

    bool isElevatingFor1000_ = false;

    bool isElevatingFor1500_ = false;

    bool isLowering_ = false;

    bool isExpanding_ = false;

    bool isExpandingByDefault_ = false;

    bool isShrinking_ = false;

    bool isShrinkingByDefault_ = false;

    bool hasOrderedExpandingByDefault_ = false;

    enum PROCESS_FLAG
    {
        STANDBY,
        START,
        START_SHEET,
        RELOAD_AND_APPROACH_POLE,
        HANG_OUT_TOWEL,
        LEAVE_POLE,
        PREPARE_TO_RETURN,
        RETURN,
        END
    } processFlag_ = STANDBY;

    enum ACTION_FLAG
    {
        NONE,
        GRIP_TOWEL,
        RELEASE_TOWEL,
        LIFT_MAGAZINE,
        DROP_MAGAZINE,
        SWING_DOWN,
        SWING_UP,
        LOCK,
        UNLOCK
    } actionFlag_ = LIFT_MAGAZINE;

private:

    const long
        COUNT_1500 = 9999,
        COUNT_1000 = 800; 

    const uint8_t
        PIN_LIMIT_ELEVATE_BOTTOM = A3,
        PIN_LIMIT_ELEVATE_TOP = 36,
        PIN_LIMIT_EXPAND = 35,
        PIN_LIMIT_EXPAND_BY_DEFAULT = 38,
        PIN_LIMIT_SHRINK = 34,
        PIN_LIMIT_POLE_GUIDE_LEFT = 33,
        PIN_LIMIT_POLE_GUIDE_RIGHT = 32,
        PIN_LIMIT_CHASSIS_GUIDE_LEFT = 31,
        PIN_LIMIT_CHASSIS_GUIDE_RIGHT = 40,
        PIN_LIMIT_HAND_GUIDE = 41;

    const Vector2
        DIST_MOVE_BACKWARD = {0, 400},
        DIST_MOVE_BACKWARD_SLIGHTLY = {0, 200},
        VIA_POINT_1TO2 = {2950+200, 3700+300};

    const double KP_ELEVATE = 0.15;

private:

    Encoder elevatorEncorder_;

    Motor elevatorMotor_, extenderMotor_;

    Solenoid swingSolenoid_,  gripperSolenoid_, lifterSolenoid_;
    
    vector<vector<vector<Route>>> routeSZ2ToTowel_, routeTowelToSZ2_;

    vector<vector<vector<Vector2>>> pointPoleBase_;

    vector<vector<vector<Vector2>>> pointHangOut_;
};

extern Towel towel;