#include "action.hpp"
#include "base.hpp"
#include "chassis.hpp"
#include "definition.hpp"
#include "i2c_lcd.hpp"
#include "odometry.hpp"
#include "sbdbt.hpp"
#include "tape_led.hpp"

Action::Action() :
timer_(7)
{}

void Action::reset()
{
    timer_.reset();
    isFollowing_ = false;
    pathIndex_ = 0;
    yawTarget_ = 0;
}

bool Action::follow(Route& route, PAUSE_MODE mode)
{
    const double KP_ROTATE = 20;
    static bool isPaused = false;

    if(isFollowing_)
    {
        if(isPaused)
        {
            tape.set(COLOR_WHITE);
            if(bt.receive() == COM_CODE::RESUME) isPaused = false;
            return false;
        }

        tape.set(COLOR_YELLOW);
        if(mode==DEFAULT || mode==NO_RESUME) bt.transmit(COM_CODE::PAUSE);

        double devYaw = getYawDeviation();

        if(route.isCloseToEnd(pathIndex_))
        {
            if(pathIndex_ == route.getSize()-1)
            {
                if(fabs(devYaw)<=5)
                {
                    if(mode==DEFAULT) bt.transmit(COM_CODE::RESUME);
                    chassis.brake();
                    lcd.clear(0x1d, 3);
                    pathIndex_ = 0;
                    isFollowing_ = false;
                    return true;
                }
            }
            else
            {
                ++pathIndex_;
                return false;
            }
        }

        Vector2 vTrans = route.getVelocity(pathIndex_);
        double vRotate = constrain(KP_ROTATE*fabs(devYaw), 0, 300)*setSign(devYaw, 2);

        lcd.print(0x1d, signOf(vTrans.x));
        lcd.print(0x1e, signOf(vTrans.y));
        lcd.print(0x1f, signOf(vRotate));
        
        chassis.drive(vTrans, vRotate);
    }
    else
    {
        if(mode==DEFAULT || mode==NO_RESUME)
        {
            bt.flush();
            while(1)
            {
                if(bt.receive() == COM_CODE::PAUSE)
                {
                    isPaused = true;
                    break;
                }
                if(timer_.wait(150))
                {
                    isPaused = false;
                    break;
                }
            }
            timer_.reset();
        }
        pathIndex_ = 0;
        isFollowing_ = true;
    }

    return false;
}

bool Action::moveTo(Vector2 target, Vector2 vTransMax)
{
    Vector2 devPos = target - odom.getNow();
    double devYaw = getYawDeviation();

    if(isInRange(devPos, {10,10}) && fabs(devYaw)<=2)
    {
        chassis.brake();
        lcd.clear(0x1d, 3);
        return true;
    }

    Vector2 vTrans =
    {
        (fabs(devPos.x)<=150 ? 100 : vTransMax.x)*setSign(devPos.x, 10),
        (fabs(devPos.y)<=150 ? 100 : vTransMax.y)*setSign(devPos.y, 10),
    };

    double vRotate = 100*setSign(devYaw, 2);

    chassis.drive(vTrans, vRotate);

    return false;
}

bool Action::moveTo(Vector2 target, double vTransBase)
{
    return moveTo(target, {vTransBase, vTransBase});
}

void Action::move(Vector2 vTrans, double yawErrorAllowable)
{
    double vRotate = 100*setSign(getYawDeviation(), yawErrorAllowable);

    chassis.drive(vTrans, vRotate);
}

void Action::setYawTarget(double yawTarget)
{
    yawTarget_ = yawTarget;
}

bool Action::isYawDeviationAllowable(double range)
{
    return fabs(getYawDeviation())<=range;
}

double Action::getYawDeviation()
{
    double dev = yawTarget_ - odom.getYaw();

    if(dev>180) dev -= 360;
    else if(dev<-180) dev += 360;

    return dev;
}

Action action;