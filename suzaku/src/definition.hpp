#pragma once

#include <Arduino.h>
#include "component/vector2.hpp"

#define toRad(deg) (deg/180*PI)
#define toDeg(rad) (rad/PI*180)

extern bool isInRange(Vector2, Vector2);

extern bool isClose(Vector2, Vector2, Vector2);

extern int setSign(double, double);

template<typename T> String signOf(T val)
{
    return (val==0) ? "0" : ((val>0) ? "+" : "-");
}