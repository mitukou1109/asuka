#include "towel.hpp"
#include "../action.hpp"
#include "../base.hpp"
#include "../controller.hpp"
#include "../i2c_lcd.hpp"
#include "../odometry.hpp"
#include "../tape_led.hpp"
#include "tshirt.hpp"

Towel::Towel() :
setPoint_({
    {       
        {1900,3550},
        {950,4600},
        {950,4500},
        {2100,4525} //+1150
    },
    {
        {-1300,3550},
        {-2275,4600},
        {-2275,4500},
        {-1125,4525} //+1150
    }
}),
SZToTowelRoute_({
    { {3600,0} , setPoint_[0][1] },
    { {-3600,0} , setPoint_[1][1] }
}),
HZToTowelRoute_({
    { {0,0} , setPoint_[0][1] },
    { {0,0} , setPoint_[1][1] }
}),
TowelToHZRoute_({
    { setPoint_[0][3] , {setPoint_[0][3].x,5200} , {-900,5200} , {-900,3300} , {400,3300} , {200,200} },
    { setPoint_[1][3] , {setPoint_[1][3].x,5200} , {900,5200} , {900,3300} , {-400,3300} , {-200,200} }
}),
timer_(base.timer_),
flipperSolenoid_(7),
gripperSolenoid_(5,6)
{}

bool Towel::work()
{
    if(flag_ == FLAG::GO_SZ)
    {
        lcd.print(0x10, "GO", 8);
        tshirt.lowerWing();
        if(action.follow(SZToTowelRoute_[ZONE]))
        {
            flag_ = FLAG::SET_2;
        }
    }
    if(flag_ == FLAG::GO)
    {
        lcd.print(0x10, "GO", 8);
        tshirt.lowerWing();
        if(action.follow(HZToTowelRoute_[ZONE]))
        {
            flag_ = FLAG::SET_2;
        }
    }
    else if(flag_ == FLAG::SET_1)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        tshirt.lowerWing();
        if(action.moveTo(setPoint_[ZONE][0], 300))
        {              
            flag_ = FLAG::SET_2;
        }
    }
    else if(flag_ == FLAG::SET_2)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        tshirt.lowerWing();
        if(action.moveTo(setPoint_[ZONE][1], 400))
        {                                      
            if(timer_.wait(500))
            {
                flag_ = FLAG::FLIP;
            }
        }
    }
    else if(flag_ == FLAG::FLIP)
    {
        lcd.print(0x10, "FLIP", 8);
        tape.blink(COLOR_BLUE);
        openFlipper();
        if(timer_.wait(500))
        {
            flag_ = FLAG::HOOK;
        }
    }
    else if(flag_ == FLAG::HOOK)
    {
        lcd.print(0x10, "HOOK", 8);
        tape.blink(COLOR_YELLOW);
        if(action.moveTo(setPoint_[ZONE][2], 200))
        {
            releaseTowel();
            flag_ = FLAG::UNFOLD;
        }
    }
    else if(flag_ == FLAG::UNFOLD)
    {
        lcd.print(0x10, "UNFOLD", 8);
        tape.blink(COLOR_YELLOW);
        if(action.moveTo(setPoint_[ZONE][3], 400))
        {
            flag_ = FLAG::RETURN;
        }
    }
    else if(flag_ == FLAG::RETURN)
    {
        lcd.print(0x10, "RETURN", 8);
        tshirt.adjustWing(tshirt.COUNT_WING_HOOK);
        if(action.follow(TowelToHZRoute_[ZONE]))
        {
            return true;
        }
    }

    return false;
}

bool Towel::test()
{
    if(flag_ == FLAG::RETURN)
    {   
        return true;
    }
    else
    {
        work();
    }

    return false;
}

void Towel::initialize()
{

}

void Towel::reset()
{
    foldFlipper();
    freeTowel();
}

void Towel::start()
{
    foldFlipper();
    gripTowel();

    if(base.isOnTest())
    {
        odom.correct(setPoint_[ZONE][1]);
        flag_ = FLAG::FLIP;
        return;
    }

    if(MODE_TSHIRT_N_TOWEL)
    {
        flag_ = FLAG::SET_1;
    }
    else
    {
        action.setYawTarget(180);
        tshirt.releaseHanger();
        if(FROM_SZ)
        {
            odom.correct({ZONE_RED ? 3600 : -3600, 0});
            flag_ = FLAG::GO_SZ;
        }
        else
        {
            flag_ = FLAG::GO;
        }
    }
}

void Towel::debug()
{
    
}

void Towel::openFlipper()
{
    flipperSolenoid_.open();
}

void Towel::foldFlipper()
{
    flipperSolenoid_.close();
}

void Towel::gripTowel()
{
    gripperSolenoid_.open();
}

void Towel::releaseTowel()
{
    gripperSolenoid_.close();
}

void Towel::freeTowel()
{
    gripperSolenoid_.free();
}

Towel towel;