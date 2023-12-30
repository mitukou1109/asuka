#pragma once

class VelocityProfile
{
public:

    void initialize(double, double, double, double, double, double);

    double getVelocity(double);

private:

    double pathLength_, aAccel_, aDecel_, vMax_, vStart_, vEnd_;
    
    double distanceAccel_, distanceDecel_, distanceConst_;
};