#include "path.hpp"
#include <Arduino.h>
#include <cmath>
#include "../definition.hpp"
#include "../../private/user_interface.hpp"


Path::Path(Vector2 pointStart, Vector2 pointEnd) :
    pointStart_(pointStart),
    pointEnd_(pointEnd)
{
    Vector2 dev = pointEnd_ - pointStart_;
    baseDirection_ = dev.arg();
    pathLength_ = dev.norm();
}

void Path::initializeProfile()
{
    profile_.initialize(pathLength_, A_ACCEL, A_DECEL, V_MAX, V_START, V_END);
}

double Path::getDeviation(Vector2 point)
{
    if(ui[ZONE] == ZONE_BLUE) point.x *= -1;
    Vector2 dev = pointStart_ - point;
    return dev.norm()*sin(-baseDirection_+dev.arg());
}

double Path::getDistance(Vector2 point)
{
    if(ui[ZONE] == ZONE_BLUE) point.x *= -1;
    Vector2 dev = point - pointStart_;
    return dev.norm()*cos(-baseDirection_+dev.arg());
}

Vector2 Path::getProfiledVelocity(Vector2 point)
{   
    double vBase = profile_.getVelocity(getDistance(point));
    double vRect = constrain(KP_RECT*getDeviation(point), -800, 800);
    return vectorizeVelocity(vBase, vRect);
}

Vector2 Path::vectorizeVelocity(double vBase, double vRect)
{
    double vComp = hypot(vBase, vRect);
    double phi = baseDirection_ + atan2(vRect, vBase);

    Vector2 vOut =
    {
        vComp*cos(phi),
        vComp*sin(phi)
    };

    return ui[ZONE]==ZONE_BLUE ? ~vOut : vOut;
}

bool Path::isCloseToEnd(Vector2 point)
{
    return isClose(point, ui[ZONE]==ZONE_BLUE ? ~pointEnd_ : pointEnd_, {25,25});
}