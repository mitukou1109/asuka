#pragma once

#include "../common/component/my_timer.hpp"
#include "../common/component/route.hpp"

class Base
{
public:

    Base();

    void initialize();

    void run();

    void reset();

private:

    void standby();

    void work();

    void getStarted();

    void exit();

    void quit();

    void suspend();

private:

    bool isBusy_ = false;

private:

    MyTimer generalPurposeTimer_;
};

extern Base ikaruga;