#pragma once

#include <initializer_list>
#include <vector>
#include "path.hpp"
#include "vector2.hpp"

using std::vector;
using std::initializer_list;

class Route
{
public:

    Route(vector<Vector2>);

    Route(initializer_list<Vector2>);

    Vector2 getVelocity(int, Vector2);

    bool isCloseToEnd(int, Vector2);

    size_t size();

private:

    void generatePath(vector<Vector2>);

    vector<Path> pathList_;
};