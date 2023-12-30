#pragma once

#include "component/vector2.hpp" 
#include "component/motor.hpp"

class Chassis
{
public:

    Chassis();

    void drive(Vector2, double);

    void brake();

    void debug(bool(&)[24]);

    void stopDebug();

private:

    Motor wheels_[4];

};

extern Chassis chassis;