#pragma once

#include <vector>
#include "../common/component/encoder.hpp"
#include "../common/component/motor.hpp"
#include "../common/component/my_timer.hpp"
#include "../common/component/path.hpp"
#include "../common/component/route.hpp"
#include "../common/component/solenoid.hpp"
#include "../common/component/vector2.hpp"

using std::vector;

class Collection
{
    friend void raiseTimeoutFlag();

public:
    
    Collection();
    
    void initialize();

    bool getStarted();

    bool work(MyTimer&);

    void stopTimeoutTimer();

    void moveBackMechanismToInitialPosition();

    void debug(bool(&)[24]);

    void stopDebug();

    void reset();

private:

    void checkActionRegularly();

    bool extendArm(int);

    bool shortenArm();

    void stopExtendAndShorten();

    void stretchCatcher();

    void withdrawCatcher();
    
    void catchTowel();

    void releaseTowel();

    void liftGate();

    void dropGate();

    void openGate();

    void closeGate();

    void pushWall();

    void pullWall();

    bool extrudeTowel();

    bool storeTowel();

    void stopExtrude();

    void checkSensorStatus();

private:

    int seconds_ = 0;

    bool timeoutFlag_ = false;

    bool somethingWentWrong_ = false;

    char zone_ = 0;

    char side_ = 0;

    char detectionRange_ = 0;

    bool isSameColor_ = false;

    bool isOmeletteRice_ = false;

    bool hasSetHeight_ = false;

    bool hasExtended_ = false;

    bool catched_ = false;

    bool hasWithdrawn_ = true;

    bool hasCorrected_ = false;

    bool hasExamined_ = false;

    bool notCollecting_ = true;

    bool isMyTowel_ = false;

    bool timeoutTimerIsStopped_ = true;

    int collectFlagTimes_ = false;

    int removeFlagTimes_ = false;

    bool modePull_ = false;

    int position_ = 0;

    int positionPrev_ = 0;

    double point_ = false;

    double originalPoint_ = false;

    bool isElevating_ = false;

    bool isLowering_ = false;

    enum PROCESS_FLAG
    {
        STANDBY,
        START,
        DETECT,
        COLLECT,
        REMOVE,
        LEAVE,
        PREPARE_TO_RETURN,
        RETURN,
        DISCHARGE,
        END,
    } processFlag_ = STANDBY;

    enum ACTION_FLAG
    {
        NONE,
        CATCH,
        RELEASE,
        STRETCH,
        WITHDRAW,
        EXTEND,
        SHORTEN,
        LIFT_GATE,
        DROP_GATE,
        OPEN_GATE,
        EXTRUDE,
        MOVE,
    } actionFlag_ = EXTEND;

    enum class TOWEL_STATUS
    {
        MOVE,
        PULL,
        CATCH,
        REMOVE,
        ERR_NOTOWEL,
        ERR_NODATA,
    };

private:

    void makeARequestToSend();

    void withdrawARequestToSend();

    TOWEL_STATUS getTowelPosition();

    bool examineWhetherThisTowelIsMine();

private:

    const uint8_t
        PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE = A5,
        PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE = A6,
        PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT = 39,
        PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT = A4,
        PIN_LIMIT_EXTRUDE = A7,
        PIN_LIMIT_STORE = A8,
        PIN_LIMIT_ARM_EXTEND = A9,
        PIN_LIMIT_ARM_SHORTEN = A10,
        PIN_SERIAL_THINKPAD_RTS = 13,
        PIN_UV_LED = 26,
        PIN_PHOTO_TRANSISTOR =  A11;

    const int TIME_LIMIT = 20; // seconds

    const int photoTransistorThreshold_ = 880;

    const double RATIO_OF_LENGTH_COUNT = 950./4750.;

    const double KP_MOVE = 1.25;

    const double KP_EXTEND = 0.35;

private:

    UARTClass& serialThinkpad_;

    MyTimer thinkpadTimeoutTimer_;

    DueTimer timeoutTimer_;

    Encoder extenderEncorder_;

    Motor extenderMotor_;

    RelayMotor extruderRelayMotor_;

    Solenoid catcherSolenoid_, stretcherSolenoid_, lifterSolenoid_, openerSolenoid_, pusherSolenoid_;

    vector<Route> routeSZ2ToTable_;

    vector<vector<Route>> routeTableToSZ2_;
};


extern Collection collection;