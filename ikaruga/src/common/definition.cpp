#include "definition.hpp"
#include <cctype>

double toRad(double deg) { return deg/180.*PI; }
double toDeg(double rad) { return rad/PI*180.; }

bool isNumber(String str)
{
    for(char c : str)
    {
        if(c != '-' and !std::isdigit(c) and c != '.')
        {
            return false;
        }
    }
    return true;
}

bool isInRange(Vector2 vec, Vector2 threshold)
{
    return fabs(vec.x) <= threshold.x and fabs(vec.y) <= threshold.y;
}

bool isClose(Vector2 vec1, Vector2 vec2, Vector2 threshold)
{
    return isInRange(vec1-vec2, threshold);
}

int setSign(double ref, double threshold)
{
    return (fabs(ref)<=threshold) ? 0 : ((ref>0) ? 1 : -1);
}

int setSign(double ref)
{
    return (ref>0) ? 1 : -1;
}

void clearSerialBuffer(UARTClass& serial)
{
    while (serial.available() > 0)
    {
        serial.read();
    }
}