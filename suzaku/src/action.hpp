#pragma once

#include "component/my_timer.hpp"
#include "component/route.hpp"

#undef NONE
#undef DEFAULT

#define PAUSE_NONE      (Action::PAUSE_MODE::NONE)
#define PAUSE_DEFAULT   (Action::PAUSE_MODE::DEFAULT)
#define PAUSE_NO_RESUME (Action::PAUSE_MODE::NO_RESUME)

class Action
{
public:

    enum PAUSE_MODE
    {
        NONE,
        DEFAULT,
        NO_RESUME
    };

    Action();

    void reset();

    bool follow(Route&, PAUSE_MODE = DEFAULT);

    bool moveTo(Vector2, Vector2);

    bool moveTo(Vector2, double);

    void move(Vector2, double = 2);

    void setYawTarget(double);

    bool isYawDeviationAllowable(double = 2);

private:

    double getYawDeviation();

    enum COM_CODE
    {
        PAUSE = 5,
        RESUME = 6
    };

    bool isFollowing_ = false;

    int pathIndex_ = 0;

    double yawTarget_ = 0;

    MyTimer timer_;
};

extern Action action;