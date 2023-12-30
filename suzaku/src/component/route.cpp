#include "route.hpp"

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
    }
}

Vector2 Route::getVelocity(int index)
{
    return pathList_.at(index).getVelocity();
}

int Route::getSize()
{
    return pathList_.size();
}

bool Route::isCloseToEnd(int index)
{
    return pathList_.at(index).isCloseToEnd();
}