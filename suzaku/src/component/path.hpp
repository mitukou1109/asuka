#pragma once

#include "vector2.hpp"
#include "velocity_profile.hpp"

class Path
{
public:

    Path(Vector2, Vector2);

    double getDeviation(Vector2);

    double getDistance(Vector2);

    Vector2 getVelocity();

    Vector2 vectorizeVelocity(double, double);

    bool isCloseToEnd();

private:

    const double
    A_ACCEL = 3000,
    A_DECEL = 300,
    V_MAX = 2000,
    V_START = 200,
    V_END = 0,
    KP_RECT = 3;

    VelocityProfile profile_;

    Vector2 pointStart_, pointEnd_;

    double baseDirection_ = 0;
    
    double pathLength_ = 0;
};