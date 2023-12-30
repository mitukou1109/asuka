#pragma once

#include "vector2.hpp"
#include "velocity_profile.hpp"

class Path
{
public:

    Path(Vector2, Vector2);

    void initializeProfile();

    double getDeviation(Vector2);

    double getDistance(Vector2);

    Vector2 getProfiledVelocity(Vector2);

    Vector2 vectorizeVelocity(double, double);

    bool isCloseToEnd(Vector2);

private:

    double baseDirection_ = 0;
    
    double pathLength_ = 0;

private:

    const double 
        A_ACCEL = 3000,
        A_DECEL = 350,
        V_MAX = 1900,
        V_START = 200,
        V_END = 0,
        KP_RECT = 3;

private: 

    VelocityProfile profile_;

    Vector2 pointStart_, pointEnd_;
};