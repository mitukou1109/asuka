#pragma once

#include "../component/route.hpp"
#include "../component/my_timer.hpp"
#include "../component/solenoid.hpp"

class Towel
{
public:

    Towel();

    bool work();

    bool test();

    void initialize();

    void reset();

    void start();

    void debug();

    void openFlipper();

    void foldFlipper();

    void gripTowel();

    void releaseTowel();

    void freeTowel();

private:

    enum FLAG
    {
        GO_SZ,
        GO,
        SET_1,
        SET_2,
        FLIP,
        HOOK,
        UNFOLD,
        BACK,
        RETURN
    } flag_ = GO;

    Vector2 setPoint_[2][4];

    Route SZToTowelRoute_[2], HZToTowelRoute_[2], TowelToHZRoute_[2];

    MyTimer& timer_;

    Solenoid flipperSolenoid_, gripperSolenoid_;
};

extern Towel towel;