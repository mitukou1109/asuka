#pragma once

#include <Arduino.h>

#undef min
#undef max

#define DIR_BRAKE RelayMotor::DIRECTION::BRAKE
#define DIR_CCW RelayMotor::DIRECTION::CCW
#define DIR_CW RelayMotor::DIRECTION::CW

class Motor
{
public:

    Motor(int);

    static void initialize();

    void drive(int);

    void setPWM(int);

    void brake();

private:

    int id_;

    static USARTClass& serialMDC_;
};

class RelayMotor
{
public:

    enum DIRECTION
    {   
        BRAKE = 2,
        CCW = 1,
        CW = 0
    };
    
    RelayMotor(int, int);

    void drive(int);

    void brake();

private:

    int pinCCW_, pinCW_;
};