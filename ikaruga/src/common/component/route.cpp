#include "route.hpp"
#include <Arduino.h>

namespace std
{
    void __throw_out_of_range(char const*) {}
    void __throw_bad_alloc() {}
    void __throw_length_error(char const*) {}
}

Route::Route(vector<Vector2> pointList)
{
    generatePath(pointList);
}

Route::Route(initializer_list<Vector2> list)
{
    vector<Vector2> pointList;
    for(auto point : list)
    {
        pointList.push_back(point);
    }
    generatePath(pointList);
}

void Route::generatePath(vector<Vector2> pointList)
{
    for(int i=0; i<pointList.size()-1; i++)
    {
        pathList_.emplace_back(pointList.at(i), pointList.at(i+1));
        pathList_.at(i).initializeProfile();
    }
}

Vector2 Route::getVelocity(int index, Vector2 point)
{
    return pathList_.at(index).getProfiledVelocity(point);
}

bool Route::isCloseToEnd(int index, Vector2 point)
{
    return pathList_.at(index).isCloseToEnd(point);
}

size_t Route::size()
{
    return pathList_.size();
}