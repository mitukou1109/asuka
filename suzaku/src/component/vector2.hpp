#pragma once

class Vector2
{
public:

    Vector2(double, double);

    double arg();

    double norm();

    Vector2 operator - (Vector2 vec)
    {
        return {this->x - vec.x, this->y - vec.y};
    }

    double x;
    
    double y;
};