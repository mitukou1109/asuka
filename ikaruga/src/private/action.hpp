#pragma once

#include "../common/component/my_timer.hpp"
#include "../common/component/route.hpp"
#include "../common/component/vector2.hpp"

class Action
{
public:

    Action();

    enum PAUSE_MODE
    {
        NONE,
        SEND,
        RECEIVE,
        MUTUAL
    };

    bool follow(Route &route, PAUSE_MODE mode=NONE);

    bool correctMyPosition(Vector2 point, Vector2 allowableError={10, 10});

    void move(Vector2 vTransration);

    bool moveTo(Vector2 point, double vTranslationMax=400, Vector2 allowableError={5, 5});

    bool moveToward(Vector2 point, double vTranslationMax=400, Vector2 allowableError={25, 25});

    double getYawDeviation() const;

    void setYawTarget(double yawTarget);

    bool isPaused() const;

    void reset();

private:

    bool isFollowing_ = false;

    bool isPaused_ = false;

    int pathIndex_ = 0;

    double yawTarget_ = 0;

private:

    const double 
        KP_ROTATION = 15,
        KP_TRANSLATION = 1.5;

private:

    MyTimer delayTimer_;
};

extern Action action;