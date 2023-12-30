#include "collection.hpp"
#include "../action.hpp"
#include "../base.hpp"
#include "../chassis.hpp"
#include "../controller.hpp"
#include "../definition.hpp"
#include "../i2c_lcd.hpp"
#include "../odometry.hpp"
#include "../tape_led.hpp"
#include "tshirt.hpp"
#include "sheet.hpp"

Collection::Collection() :
collectPoint_(
    {0,750}
),
HZPoint_({
    {-3450,100},
    {3450,100}
}),
collectionToHZRoute_({ 
    { collectPoint_, HZPoint_[0] },
    { collectPoint_, HZPoint_[1] }
}),
SZ1ToHZRoute_({
    { {0,0} , HZPoint_[0] },
    { {0,0} , HZPoint_[1] }
}),
timer_(base.timer_),
armMotor_(44,45),
conveyorMotor_(42,43),
mainGateSolenoid_(0),
subGateSolenoid_(1)
{}

bool Collection::work()
{
    if(flag_ == FLAG::GO_SZ)
    {
        lcd.print(0x10, "GO", 8);
        if(action.follow(SZ1ToHZRoute_[ZONE], PAUSE_NONE))
        {
            return true;
        }
    }
    else if(flag_ == FLAG::EXTEND)
    {
        lcd.print(0x10, "EXTEND", 8);
        tape.set(COLOR_BLUE);
        if(extendArm())
        {
            flag_ = FLAG::SET_1;
        }
    }
    else if(flag_ == FLAG::SET_1)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        if(setPositionToCollect(collectPoint_))
        {
            flag_ = FLAG::CLOSE;
        }
    }
    else if(flag_ == FLAG::CLOSE)
    {
        lcd.print(0x10, "CLOSE", 8);
        tape.set(COLOR_BLUE);
        closeGate();
        if(timer_.wait(500))
        {
            flag_ = FLAG::COLLECT;
        }
    }
    else if(flag_ == FLAG::COLLECT)
    {
        lcd.print(0x10, "COLLECT", 8);
        tape.set(COLOR_BLUE);
        driveConveyor(ZONE_RED ? DIR_CCW : DIR_CW);
        if(shrinkArm())
        {
            flag_ = FLAG::WAIT;
        }
    }
    else if(flag_ == FLAG::WAIT)
    {
        lcd.print(0x10, "WAIT", 8);
        tape.set(COLOR_BLUE);
        driveConveyor(ZONE_RED ? DIR_CCW : DIR_CW);
        if(timer_.wait(250))
        {
            tshirt.gripHanger();
            sheet.extendHook();
            sheet.releaseSheet();
            action.setYawTarget(ZONE_RED ? 180 : -180);
            openSubGate();
            flag_ = FLAG::GO;
        }
    }
    else if(flag_ == FLAG::GO)
    {
        lcd.print(0x10, "GO", 8);
        tshirt.adjustWing(tshirt.COUNT_WING_HOOK);
        sheet.shrinkSlider();
        if(!digitalRead(PIN_LIMIT_CHASSIS_TOUCH)) odom.correctY(0);
        if(timer_.wait(250)) stopConveyor();
        if(action.follow(collectionToHZRoute_[ZONE], PAUSE_NONE))
        {
            timer_.reset();
            tshirt.stopWing();
            flag_ = FLAG::SET_2;
        }
    }
    else if(flag_ == FLAG::SET_2)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        if(setPositionToDrop(HZPoint_[ZONE]))
        {
            flag_ = FLAG::OPEN;
        }
    }
    else if(flag_ == FLAG::OPEN)
    {
        lcd.print(0x10, "OPEN", 8);
        tape.set(COLOR_BLUE);
        openGate();
        if(timer_.wait(500))
        {
            flag_ = FLAG::DROP;
        }
    }
    else if(flag_ == FLAG::DROP)
    {
        lcd.print(0x10, "DROP", 8);
        tape.blink(COLOR_BLUE);
        driveConveyor(ZONE_RED ? DIR_CW : DIR_CCW);
        if(timer_.wait(2000))
        {
            flag_ = FLAG::SET_3;
        }
    }
    else if(flag_ == FLAG::SET_3)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        if(action.moveTo(HZPoint_[ZONE], 200))
        {
            return true;
        }
    }
    
    return false;
}

bool Collection::test()
{
    if(flag_ == FLAG::GO) return true;
    else work();

    return false;
}

bool Collection::setPositionToCollect(Vector2 reference)
{
    Vector2 devPos = reference - odom.getNow();

    Vector2 vTrans =
    {
        ZONE_RED ? 100 : -100,
        (fabs(devPos.y)>=100 ? 300 : 100)*setSign(devPos.y, 10)
    };

    if(!digitalRead(ZONE_RED ? PIN_LIMIT_CHASSIS_GUIDE_RIGHT : PIN_LIMIT_CHASSIS_GUIDE_LEFT))
    {
        if(fabs(devPos.y)<=10 && action.isYawDeviationAllowable())
        {
            chassis.brake();
            return true;
        }
        else
        {
            action.move({0,vTrans.y});
        }
    }
    else
    {
        action.move(vTrans);
    }

    return false;
}

bool Collection::setPositionToDrop(Vector2 reference)
{
    Vector2 devPos = reference - odom.getNow();

    Vector2 vTrans =
    {
        ZONE_RED ? -200 : 200,
        (fabs(devPos.y)>=100 ? 300 : 100)*setSign(devPos.y, 10)
    };

    if(fabs(devPos.y)<=50)
    {
        if(!digitalRead(ZONE_RED ? PIN_LIMIT_CHASSIS_GUIDE_RIGHT : PIN_LIMIT_CHASSIS_GUIDE_LEFT))
        {
            if(fabs(devPos.y)<=10 && action.isYawDeviationAllowable())
            {
                chassis.brake();
                return true;
            }
            else
            {
                action.move({0,vTrans.y});
            }
        }
        else
        {
            action.move(vTrans);
        }
    }
    else
    {
        action.move({0,vTrans.y});
    }

    return false;
}

void Collection::initialize()
{
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_LEFT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_RIGHT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_TOUCH, INPUT);
    pinMode(PIN_LIMIT_ARM_EXTEND_RED, INPUT);
    pinMode(PIN_LIMIT_ARM_EXTEND_BLUE, INPUT);
    pinMode(PIN_LIMIT_ARM_SHRINK, INPUT);
}

void Collection::reset()
{
    stopArm();
    stopConveyor();
    openGate();
}

void Collection::start()
{
    if(FROM_SZ)
    {
        action.setYawTarget(180);
        flag_ = FLAG::GO_SZ;
    }
    else
    {
        action.setYawTarget(0);
        flag_ = FLAG::EXTEND;
    }
}

void Collection::debug()
{
    lcd.print(0x00, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_LEFT));
    lcd.print(0x01, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_RIGHT));
    lcd.print(0x02, digitalRead(PIN_LIMIT_CHASSIS_TOUCH));
    lcd.print(0x03, digitalRead(PIN_LIMIT_ARM_EXTEND_RED));
    lcd.print(0x04, digitalRead(PIN_LIMIT_ARM_EXTEND_BLUE));
    lcd.print(0x05, digitalRead(PIN_LIMIT_ARM_SHRINK), 2);
}

bool Collection::extendArm()
{
    if(digitalRead(ZONE_RED ? PIN_LIMIT_ARM_EXTEND_RED : PIN_LIMIT_ARM_EXTEND_BLUE))
    {
        stopArm();
        return true;
    }
    else
    {
        armMotor_.drive(ZONE_RED ? DIR_CCW : DIR_CW);
        return false;
    }
}

bool Collection::shrinkArm()
{
    if(digitalRead(PIN_LIMIT_ARM_SHRINK))
    {
        stopArm();
        return true;
    }
    else
    {
        armMotor_.drive(ZONE_RED ? DIR_CW : DIR_CCW);
        return false;
    }
}

void Collection::stopArm()
{
    armMotor_.brake();
}

void Collection::driveConveyor(int direction)
{
    conveyorMotor_.drive(direction);
}

void Collection::stopConveyor()
{
    conveyorMotor_.brake();
}

void Collection::openMainGate()
{
    mainGateSolenoid_.close();
}

void Collection::closeMainGate()
{
    mainGateSolenoid_.open();
}

void Collection::openSubGate()
{
    subGateSolenoid_.close();
}

void Collection::closeSubGate()
{
    subGateSolenoid_.open();
}

void Collection::openGate()
{
    openMainGate();
    openSubGate();
}

void Collection::closeGate()
{
    closeMainGate();
    closeSubGate();
}

Collection collection;