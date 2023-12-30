#pragma once

#include "component/encoder.hpp"
#include "component/my_timer.hpp"
#include "component/vector2.hpp"

class Odometry
{
    friend void raiseRefreshFlag();

public:

    const int
    PIN_RESET = 49;

    Odometry();

    Vector2 getNow() const;

    double getYaw() const;

    void initialize();

    void reset();

    void resetGyro();

    bool refresh();

    void correct(Vector2);

    void correctX(double);

    void correctY(double);

    void setYawAdd(double);

    void printData();

private:

    enum AXIS
    {
        X = 0,
        Y = 1
    };

    const double
    ODOMETER_C = 2*3.14*25.4,
    ODOMETER_CPR = 512*4,
    REFRESH_RATE = 5;

    void readGyro();

    void flushGyro();

    Encoder odometer_[2];

    USARTClass& serialGyro_;

    DueTimer refreshTimer_;

    MyTimer timeoutTimer_;

    Vector2 now_ = {0,0};

    double yaw_ = 0, yawAdd_ = 0;

    volatile bool refreshFlag_ = false;
};

extern Odometry odom;