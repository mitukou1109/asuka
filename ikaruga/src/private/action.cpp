#include "action.hpp"
#include "../common/chassis.hpp"
#include "../common/definition.hpp"
#include "../common/i2c_lcd.hpp"
#include "../common/odometry.hpp"
#include "../common/sbdbt.hpp"
#include "../common/tape_led.hpp"
#include "user_interface.hpp"

Action::Action() : 
    delayTimer_(7)
{}

bool Action::follow(Route &route, PAUSE_MODE mode)
{
    if (isFollowing_)
    {
        if (isPaused_)
        {
            tape.set(COLOR_WHITE);
            if (bt.read() == COM_CODE::RESUME) isPaused_ = false;
            return false;
        }

        tape.set(COLOR_YELLOW);
        Vector2 now = odom.getNow();
        double devYaw = getYawDeviation();

        if ((mode == SEND or mode == MUTUAL) and pathIndex_ <= 1) bt.write(COM_CODE::PAUSE);
        else                                  bt.write(COM_CODE::RESUME);
        if (route.isCloseToEnd(pathIndex_, now))
        {
            if (pathIndex_ == route.size()-1)
            {
                if (fabs(devYaw) <= 3)
                {
                    bt.write(COM_CODE::RESUME);
                    chassis.brake();
                    LCD.clear(0x1d, 3);
                    pathIndex_ = 0;
                    isFollowing_ = false;
                    return true;
                }
            }
            else
            {
                ++pathIndex_;
            }
        }

        Vector2 vTranslation = route.getVelocity(pathIndex_, now);
        double vRotation = constrain(fabs(devYaw)*KP_ROTATION, 0, 200)*setSign(devYaw, 1);

        LCD.print(0x1d, signOf(vTranslation.x), 1);
        LCD.print(0x1e, signOf(vTranslation.y), 1);
        LCD.print(0x1f, signOf(vRotation), 1);

        chassis.drive(vTranslation, vRotation);
    }
    else
    {
        if (mode == RECEIVE or mode == MUTUAL)
        {
            bt.flush();
            tape.set(COLOR_BLUE);
            while (1)
            {
                if (bt.read() == COM_CODE::PAUSE)
                {
                    isPaused_ = true;
                    delayTimer_.reset();
                    break;
                }
                if (delayTimer_.wait(200))
                {
                    isPaused_ = false;
                    break;
                }
            }
        }
        pathIndex_ = 0;
        isFollowing_ = true;
    }
    return false;
}

bool Action::correctMyPosition(Vector2 point, Vector2 allowableError)
{
    if (ui[ZONE] == ZONE_BLUE) point.x *= -1;
    Vector2 now = odom.getNow();
    double devYaw = getYawDeviation();

    if (isClose(now, point, allowableError))
    {
        if (fabs(devYaw) <= 2)
        {
            if (delayTimer_.wait(100))
            {
                chassis.brake();
                LCD.clear(0x1d, 3);
                return true;
            }
        }
        else
        {
            delayTimer_.reset();
        }
    }

    Vector2 vTranslation =
    {
        70*setSign((point - now).x, allowableError.x),
        70*setSign((point - now).y, allowableError.y)
    };
    double vRotation = 30*setSign(devYaw, 2);

    LCD.print(0x1d, signOf(vTranslation.x), 1);
    LCD.print(0x1e, signOf(vTranslation.y), 1);
    LCD.print(0x1f, signOf(vRotation), 1);

    chassis.drive(vTranslation, vRotation);
    return false;
}

void Action::move(Vector2 vTranslation)
{
    if (ui[ZONE] == ZONE_BLUE) vTranslation.x *= -1;
    double devYaw = getYawDeviation();
    double vRotation = constrain(fabs(devYaw)*KP_ROTATION, 0, 150)*setSign(devYaw, 1);
    chassis.drive(vTranslation, vRotation);
}

bool Action::moveTo(Vector2 point, double vTranslationMax, Vector2 allowableError)
{
    if (ui[ZONE] == ZONE_BLUE) point.x *= -1;
    Vector2 devPos = point - odom.getNow();
    double devYaw = getYawDeviation();

    if (isInRange(devPos, {50, 50}))
    {
        if (ui[ZONE] == ZONE_BLUE)
        {
            point.x *= -1;
        }

        if (correctMyPosition(point, allowableError))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    Vector2 vTranslation =
    {
        constrain(fabs(devPos.x)*KP_TRANSLATION, 0, vTranslationMax)*setSign(devPos.x, 25),
        constrain(fabs(devPos.y)*KP_TRANSLATION, 0, vTranslationMax)*setSign(devPos.y, 25)
    };
    double vRotation = constrain(fabs(devYaw)*KP_ROTATION, 0, 150)*setSign(devYaw, 1);

    LCD.print(0x1d, signOf(vTranslation.x), 1);
    LCD.print(0x1e, signOf(vTranslation.y), 1);
    LCD.print(0x1f, signOf(vRotation), 1);

    chassis.drive(vTranslation, vRotation);
    return false;
}

bool Action::moveToward(Vector2 point, double vTranslationMax, Vector2 allowableError)
{
    if (ui[ZONE] == ZONE_BLUE)
    {
        point.x *= -1;
    }

    Vector2 devPos = point - odom.getNow();
    double devYaw = getYawDeviation();

    if (isInRange(devPos, allowableError))
    {
        if (fabs(devYaw) <= 3)
        {
            chassis.brake();
            LCD.clear(0x1d, 3);
            return true;
        }
    }

    Vector2 vTranslation =
    {
        constrain(fabs(devPos.x)*KP_TRANSLATION, 0, vTranslationMax)*setSign(devPos.x, allowableError.x),
        constrain(fabs(devPos.y)*KP_TRANSLATION, 0, vTranslationMax)*setSign(devPos.y, allowableError.y)
    };
    double vRotation = constrain(fabs(devYaw)*KP_ROTATION, 0, 150)*setSign(devYaw, 1);

    LCD.print(0x1d, signOf(vTranslation.x), 1);
    LCD.print(0x1e, signOf(vTranslation.y), 1);
    LCD.print(0x1f, signOf(vRotation), 1);

    chassis.drive(vTranslation, vRotation);
    return false;
}

void Action::setYawTarget(double yawTarget)
{
    yawTarget_ = yawTarget;
}

double Action::getYawDeviation() const
{
    double dev = yawTarget_ - odom.getYaw();
    if (dev > 180)
    {
        dev -= 360;
    }
    else if (dev < -180)
    {
        dev += 360;
    }
    return dev;
}

bool Action::isPaused() const
{
    return isPaused_;
}

void Action::reset()
{
    pathIndex_ = 0;
    isPaused_ = false;
    isFollowing_ = false;
}

Action action;