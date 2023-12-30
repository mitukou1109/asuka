#include "velocity_profile.hpp"
#include <Arduino.h>
#include <math.h>

void VelocityProfile::initialize(double pathLength, double aAccel, double aDecel, double vMax, double vStart, double vEnd)
{
    pathLength_ = pathLength;
    aAccel_ = aAccel;
    aDecel_ = aDecel;
    vMax_ = vMax;
    vStart_ = vStart;
    vEnd_ = vEnd;

    double tAccel = (vMax_-vStart_)/aAccel_;
    double tDecel = (vMax_-vEnd_)/aDecel_;

    distanceAccel_ = 0.5*(vStart_+vMax_)*tAccel;
    distanceDecel_ = 0.5*(vMax_+vEnd_)*tDecel;

    distanceConst_ = pathLength_-(distanceAccel_+distanceDecel_);
    if(distanceConst_ < 0)
    {
        distanceConst_ = 0;
        vMax_ = sqrt((2*pathLength_*aAccel_*aDecel_+aDecel_*pow(vStart_,2)+aAccel_*pow(vEnd_,2))/(aAccel_+aDecel_));
        tAccel = (vMax_-vStart_)/aAccel_;
        tDecel = (vMax_-vEnd_)/aDecel_;
        distanceAccel_ = 0.5*(vStart_+vMax_)*tAccel;
        distanceDecel_ = 0.5*(vMax_+vEnd_)*tDecel;
    }
}

double VelocityProfile::getVelocity(double distance)
{
    if(distance < 0)
    {
        return vStart_;
    }
    else if(distance <= distanceAccel_)
    {
        return sqrt(pow(vStart_, 2)+2*aAccel_*distance);
    }
    else if(distance <= distanceAccel_+distanceConst_)
    {
        return vMax_;
    }
    else if(distance <= pathLength_)
    {
        return sqrt(pow(vMax_, 2) - 2*aDecel_*(distance-(distanceAccel_+distanceConst_)));
    }
    else
    {
        distance = constrain(distance-pathLength_, 0, distanceAccel_+distanceConst_);
        if(distance <= distanceAccel_)
        {
            return -sqrt(pow(vEnd_, 2) + 2*aAccel_*distance);
        }
        else
        {
            return -vMax_;
        }
    }
}