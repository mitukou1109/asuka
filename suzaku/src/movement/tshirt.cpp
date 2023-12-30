#include "tshirt.hpp"
#include "../action.hpp"
#include "../base.hpp"
#include "../chassis.hpp"
#include "../controller.hpp"
#include "../definition.hpp"
#include "../i2c_lcd.hpp"
#include "../odometry.hpp"
#include "../tape_led.hpp"
#include "collection.hpp"

Tshirt::Tshirt() :
setPoint_({
    {
        {2200,3600}, 
        {1600+25,3550}  //FINAL
    },
    {
        {-1000,3600}, 
        {-1600+25,3550} //FINAL
    }
}),
SZToTshirtRoute_({
    {
        { {3600,0} , setPoint_[0][0] },
        { {3600,0} , {1600,200} , setPoint_[0][1] }
    },
    {
        { {-3600,0} , setPoint_[1][0] },
        { {-3600,0} , {-1600,200} , setPoint_[1][1] }
    }
}),
HZToTshirtRoute_({
    {
        { {0,0} , setPoint_[0][0] },
        { {0,0} , {1600,200} , setPoint_[0][1] }
    },
    {
        { {0,0} , setPoint_[1][0] },
        { {0,0} , {-1600,200} , setPoint_[1][1] }
    }
}),
TshirtToHZRoute_({
    {
        { setPoint_[0][0] , {200,200} },
        { setPoint_[0][1] , {200,200} }
    },
    {
        { setPoint_[1][0] , {-200,200} },
        { setPoint_[1][1] , {-200,200} }
    }
}),
timer_(base.timer_),
wingEncoder_(3,2),
wingMotor_(6),
hangerSolenoid_(3,4)
{}

bool Tshirt::work()
{
    if(flag_ == FLAG::GO_SZ)
    {
        lcd.print(0x10, "GO", 8);
        adjustWing(COUNT_WING_HOOK);
        if(action.follow(SZToTshirtRoute_[ZONE][FINAL]))
        {
            flag_ = FLAG::SET;
        }
    }
    else if(flag_ == FLAG::GO)
    {
        lcd.print(0x10, "GO", 8);
        adjustWing(COUNT_WING_HOOK);
        if(action.follow(HZToTshirtRoute_[ZONE][FINAL]))
        {
            flag_ = FLAG::SET;
        }
    }
    else if(flag_ == FLAG::SET)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        adjustWing(COUNT_WING_HOOK);
        if(setPositionToHook())
        {
            flag_ = FLAG::LOWER;
        }
    }
    else if(flag_ == FLAG::LOWER)
    {
        lcd.print(0x10, "LOWER", 8);
        tape.set(COLOR_BLUE);
        if(lowerWing())
        {
            if(timer_.wait(200))
            {
                flag_ = FLAG::RELEASE_1;
            }
        }
    }
    else if(flag_ == FLAG::RELEASE_1)
    {
        lcd.print(0x10, "RELEASE", 8);
        tape.blink(COLOR_BLUE);
        releaseHanger();
        if(timer_.wait(200))
        {
            flag_ = FLAG::RELEASE_2;
        }
    }
    else if(flag_ == FLAG::RELEASE_2)
    {
        lcd.print(0x10, "RELEASE", 8);
        tape.set(COLOR_BLUE);
        if(adjustWing(COUNT_WING_RELEASE))
        {
            if(timer_.wait(200))
            {
                flag_ = FLAG::BACK;
            }
        }
    }
    else if(flag_ == FLAG::BACK)
    {
        lcd.print(0x10, "BACK", 8);
        tape.blink(COLOR_YELLOW);
        if(action.moveTo(setPoint_[ZONE][FINAL], 200))
        {
            if(MODE_TSHIRT_N_TOWEL) return true;
            else flag_ = FLAG::RETURN;
        }
    }
    else if(flag_ == FLAG::RETURN)
    {
        lcd.print(0x10, "RETURN", 8);
        adjustWing(COUNT_WING_HOOK);
        if(action.follow(TshirtToHZRoute_[ZONE][FINAL]))
        {
            return true;
        }
    }

    return false;
}

bool Tshirt::test()
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

bool Tshirt::setPositionToHook()
{
    Vector2 devPos = setPoint_[ZONE][FINAL] - odom.getNow();
    
    Vector2 vTrans =
    {
        100*setSign(devPos.x, 10),
        FINAL ? 100 : 200
    };

    if(!digitalRead(PIN_LIMIT_POLE_TOUCH))
    {
        if(fabs(devPos.x)<=10)
        {
            if(action.isYawDeviationAllowable(1))
            {
                chassis.brake();
                return true;
            }
            else
            {
                action.move({0,0}, 1);
            }
        }
        else
        {
            action.move({vTrans.x,0}, 1);
        }   
    }
    else
    {
        action.move(vTrans);
    }

    return false;
}

void Tshirt::initialize()
{
    pinMode(PIN_LIMIT_POLE_TOUCH, INPUT);
    pinMode(PIN_LIMIT_WING_LOWER, INPUT);
}

void Tshirt::reset()
{
    stopWing();
    freeHanger();
}

void Tshirt::start()
{
    action.setYawTarget(180);
    collection.closeMainGate();
    gripHanger();
    
    if(base.isOnTest())
    {
        odom.correct(setPoint_[ZONE][FINAL]);
        flag_ = FLAG::SET;
        return;
    }

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

void Tshirt::debug()
{
    lcd.print(0x07, digitalRead(PIN_LIMIT_POLE_TOUCH));
    lcd.print(0x08, digitalRead(PIN_LIMIT_WING_LOWER), 2);
    lcd.print(0x0a, wingEncoder_.getCount()/1000, 3);
}

bool Tshirt::adjustWing(int count)
{
    if(!isWingZeroDetected_)
    {
        lowerWing();
        return false;
    }

    int dev = count - wingEncoder_.getCount();

    if(abs(dev)<=500)
    {
        stopWing();
        return true;
    }
    else
    {
        if(dev>0)
        {
            wingMotor_.setPWM(dev>5000 ? -50 : constrain(-dev*0.01, -50, -25));
        }
        else
        {
            wingMotor_.setPWM(30);
        }
    }

    return false;
}

bool Tshirt::lowerWing()
{
    if(digitalRead(PIN_LIMIT_WING_LOWER))
    {
        stopWing();
        wingEncoder_.reset();
        isWingZeroDetected_ = true;
        return true;
    }
    else
    {
        int count = wingEncoder_.getCount();
        wingMotor_.setPWM((count>0&&count<3000) ? 50 : 30);
        return false;
    }
}

void Tshirt::stopWing()
{
    wingMotor_.brake();
}

void Tshirt::gripHanger()
{
    hangerSolenoid_.open();
}

void Tshirt::releaseHanger()
{
    hangerSolenoid_.close();
}

void Tshirt::freeHanger()
{
    hangerSolenoid_.free();
}

Tshirt tshirt;