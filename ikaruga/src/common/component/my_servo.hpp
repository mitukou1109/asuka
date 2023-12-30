#pragma once

#include <PCA9685.h>

class MyServo
{
public:

    MyServo(char);

    static void initialize();

    void set(int);

private:

    char ch_;

    static PCA9685 driver;

};