#pragma once

#include <Arduino.h>
#include "component/vector2.hpp"

#define TEST_RUN false

#define ZONE_RED 1
#define ZONE_BLUE 0

#define ROUND_PRELIMINARY 0
#define ROUND_FINAL 1

#define SIDE_FRONT 1
#define SIDE_BACK 0

#define HALFWAY 1

extern double toRad(double);
extern double toDeg(double);

extern bool isNumber(String);

extern bool isInRange(Vector2, Vector2);
extern bool isClose(Vector2, Vector2, Vector2);

extern int setSign(double, double);
extern int setSign(double);

extern void clearSerialBuffer(UARTClass&);

template<class T>
String signOf(T val) { return (val==0) ? "0" : ((val>0) ? "+" : "-"); }