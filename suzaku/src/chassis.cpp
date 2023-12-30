#include "chassis.hpp"
#include "definition.hpp"
#include "odometry.hpp"

Chassis::Chassis() :
wheels_({0,1,2,3})
{}

void Chassis::drive(Vector2 velocity, double vRotate)
{
    for(int i=0;i<4;i++)
    {
        double angle = PI/4+i*PI/2+toRad(odom.getYaw());
        int vWheel = (int)round(velocity.x*cos(angle)+velocity.y*sin(angle)+vRotate);
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

Chassis chassis;