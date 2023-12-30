#pragma once

#include <cmath>

class Vector2
{
public:

    Vector2(double, double);

    double arg();
    double norm();

    Vector2 operator-(Vector2 vec)
    {
        Vector2 result =
        {
            this->x - vec.x,
            this->y - vec.y
        };
        return result;
    }

    Vector2 operator+(Vector2 vec)
    {
        Vector2 result =
        {
            this->x + vec.x,
            this->y + vec.y
        };
        return result;
    }

    Vector2 operator~()
    {
        Vector2 vec = *this;
        vec.x *= -1;
        return vec;
    }

    double x;
    double y;
};