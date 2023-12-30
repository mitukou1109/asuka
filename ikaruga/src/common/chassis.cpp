#include "chassis.hpp"
#include "definition.hpp"
#include "odometry.hpp"
#include "../private/user_interface.hpp"

Chassis::Chassis() :
    wheels_({0,1,2,3})
{}

void Chassis::drive(Vector2 vTranslation, double vRotation)
{
    for(int i = 0; i < 4; i++)
    {
        double angle = PI/4+i*PI/2+toRad(odom.getYaw());
        int vWheel = round(vTranslation.x*cos(angle)+vTranslation.y*sin(angle)+vRotation);
        wheels_[i].drive(vWheel);
    }
}

void Chassis::brake()
{
    for(auto& wheel : wheels_)
    {
        wheel.brake();
    }
}

void Chassis::debug(bool(&status)[24])
{
    if (status[CHASSIS_DRIVE_WHEELS])              drive({0, 0}, status[TOGGLE0] ? 200 : 500);
    else if (status[CHASSIS_DRIVE_WHEELS_REVERSE]) drive({0, 0}, status[TOGGLE0] ? -200 :-500);
    else                                           brake();
}

void Chassis::stopDebug()
{
    brake();
}

Chassis chassis;