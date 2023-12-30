#pragma once

#include <initializer_list>
#include <vector>
#include "path.hpp"

using std::initializer_list;
using std::vector;

class Route
{
public:

    Route(vector<Vector2>);

    Route(initializer_list<Vector2>);

    Vector2 getVelocity(int);

    int getSize();

    bool isCloseToEnd(int);

private:

    void generatePath(vector<Vector2>);

    vector<Path> pathList_;
};