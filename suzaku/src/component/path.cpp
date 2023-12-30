#include "path.hpp"
#include <Arduino.h>
#include "../definition.hpp"
#include "../odometry.hpp"

Path::Path(Vector2 pointStart, Vector2 pointEnd) :
pointStart_(pointStart),
pointEnd_(pointEnd)
{
    Vector2 dev = pointEnd_ - pointStart_;

    baseDirection_ = dev.arg();
    pathLength_ = dev.norm();

    profile_.initialize(pathLength_, A_ACCEL, A_DECEL, V_MAX, V_START, V_END);
}

double Path::getDeviation(Vector2 point)
{
    Vector2 dev = pointStart_ - point;
    return dev.norm() * sin(-baseDirection_+dev.arg());
}

double Path::getDistance(Vector2 point)
{
    Vector2 dev = point - pointStart_;
    return dev.norm() * cos(-baseDirection_+dev.arg());
}

Vector2 Path::getVelocity()
{   
    Vector2 now = odom.getNow();

    double l = getDistance(now);
    double z = getDeviation(now);

    double vBase = profile_.getVelocity(l);
    double vRect = KP_RECT*z;

    return vectorizeVelocity(vBase, vRect);
}

Vector2 Path::vectorizeVelocity(double vBase, double vRect)
{
    double vComp = hypot(vBase, vRect);
    double phi = baseDirection_ + atan2(vRect, vBase);

    return {vComp*cos(phi), vComp*sin(phi)};
}

bool Path::isCloseToEnd()
{
    return isClose(odom.getNow(), pointEnd_, {25,25});
}