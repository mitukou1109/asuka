#include "definition.hpp"

bool isInRange(Vector2 vec, Vector2 threshold)
{
    return fabs(vec.x) <= threshold.x && fabs(vec.y) <= threshold.y;
}

bool isClose(Vector2 vec1, Vector2 vec2, Vector2 threshold)
{
    return isInRange(vec1-vec2, threshold);
}

int setSign(double ref, double threshold = 0)
{
    return (fabs(ref)<=threshold) ? 0 : ((ref>0) ? 1 : -1);
}