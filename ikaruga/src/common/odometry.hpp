#pragma once

#include "component/encoder.hpp"
#include "component/my_timer.hpp"
#include "component/vector2.hpp"

class Odometry
{
    friend void raiseRefreshFlag();

public:

    Odometry();

    void initialize();

    bool refresh();

    Vector2 getNow() const;

    double getYaw() const;

    void setYawBias(double yawBias);

    void correct(Vector2);

    void reset();

    void resetGyro();

    bool canReceiveGyroData();

    void printData();

private:

    void readYaw();

private:

    Vector2 now_ = {0,0};

    double yaw_ = 0;

    double yawBias_ = 0;

    bool receiveGyroDataFlag_ = false;

    volatile bool refreshFlag_ = false;

private:

    enum AXIS
    {
        X,
        Y
    };

    const double
        ODOMETER_C = 2*3.14*25.4,
        ENCODER_CPR = 512*4,
        REFRESH_RATE = 5;

    const uint8_t PIN_RESET_GYRO = 49;

private:

    Encoder odometer[2];

    USARTClass& serialGyro_;

    DueTimer refreshTimer_;

    MyTimer timeoutTimer_;
};

extern Odometry odom;