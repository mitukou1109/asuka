#pragma once
#include <Arduino.h>
#include "../common/component/my_timer.hpp"

namespace user_interface
{
enum STATUS
{
    RESET_ODOMETRY, CHASSIS_DRIVE_WHEELS = 0,
    INITIALIZE_MECHANISM_POSITION, CHASSIS_DRIVE_WHEELS_REVERSE = 1,
    ZONE,
    ROUND,
    COLLECTION_SIDE, TOGGLE0 = 4,
    COLLECTION_HALF_RANGE, TOGGLE1 = 5,
    COLLECTION_OMELETTE_RICE, TOGGLE2 = 6,
    COLLECTION_SAME_COLOR, TOGGLE3 = 7,
    TOWEL0, TOGGLE4 = 8,
    TOWEL1, TOGGLE5 = 9,
    TOWEL2, 
    DEBUG,
    MODE_COLLECTION,
    MODE_TOWEL,
    MODE_SHEET,
    MOTOR3_FORWARD,
    MOTOR3_REVERSE,
    MOTOR2_FORWARD,
    MOTOR2_REVERSE,
    MOTOR1_FORWARD,
    MOTOR1_REVERSE,
    MOTOR0_FORWARD,
    MOTOR0_REVERSE,
    NONE,
};
}

using namespace user_interface;

class UserInterface
{
public:

    UserInterface();

    void initialize();

    bool monitor(MyTimer& watchdogTimerForGyro);

    bool startup();

    bool escape();

    void flush();

    bool isDebugging() const;

    bool hasResetOdometry() const;

    bool isDuringEmergencyStop() const;

    bool isForbidden() const;
 
    bool isReadyToWork() const;

    bool isTestingOutWork() const;

    void printData();

    void reset();

    bool& operator[](int index)
    {
        return status_[index];
    }

private:

    bool read();

    void debug();

    void selectMode();

    void selectOption(MyTimer&);

    void resetOdometry(MyTimer&);

    void resetGyro(MyTimer&);

    void monitorGyro(MyTimer&);

private:

    bool status_[24] = {};

    bool isPushed_ = false;

    bool isDebugging_ = false;

    bool hasResetOdometry_ = false;

    bool isTestingOutWork_ = false;

    enum CHECK
    {
        NONE,
        COLLECTION,
        TOWEL,
        SHEET
    } check_ = NONE;

private:

    const uint8_t
        PIN_START1 = A0,
        PIN_START2 = 37,
        PIN_TEST_OUT = 12,
        PIN_EMS = A1,
        PIN_RESET = A2;

private:

    UARTClass &serialUI_;

    MyTimer serialUITimeoutTimer_;
};

extern UserInterface ui;