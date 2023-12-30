#include "vector2.hpp"

Vector2::Vector2(double _x, double _y)
{
    x = _x;
    y = _y;
}

double Vector2::arg()
{
    return atan2(y, x);
}

double Vector2::norm()
{
    return hypot(x, y);
}