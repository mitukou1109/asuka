#pragma once

#include <Arduino.h>

enum MOTOR_DIRECTION
{   
    BRAKE = 2,
    CCW = 1,
    CW = 0
};

class Motor
{
public:

    Motor(char);

    static void initialize();

    void drive(int);

    void setPWM(int);

    void brake();

//private:

    char id_;

    static USARTClass& serialMDC_;
};

class RelayMotor
{
public:
    
    RelayMotor(char, char);

    void drive(char);

    void brake();

//private:

    char pinCCW_, pinCW_;

};