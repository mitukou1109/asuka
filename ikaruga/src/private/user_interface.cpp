#include "user_interface.hpp"
#include "../common/i2c_lcd.hpp"
#include "../common/definition.hpp"
#include "../common/tape_led.hpp"
#include "../common/odometry.hpp"
#include "../common/chassis.hpp"
#include "../common/onboard_led.hpp"
#include "collection.hpp"
#include "towel.hpp"
#include "sheet.hpp"
#include "base.hpp"

UserInterface::UserInterface() :
    serialUI_(Serial1),
    serialUITimeoutTimer_(4)
{}

void UserInterface::initialize()
{
    serialUI_.begin(115200);
    pinMode(PIN_START1, INPUT);
    pinMode(PIN_START2, INPUT);
    pinMode(PIN_TEST_OUT, INPUT);
    reset();
}

bool UserInterface::monitor(MyTimer& timer)
{
    if (read())
    {
        if (status_[DEBUG])
        {
            tape.set(COLOR_PINK);
            isDebugging_ = true;
            debug();
        }
        else
        {
            if (isDebugging())
            {
                ikaruga.reset();
                isDebugging_ = false;
            }

            chassis.brake();
            printData();
            if (hasResetOdometry())
            {
                monitorGyro(timer);
                selectMode();
            }
            else
            {
                LCD.print(0x10, "Reset Odom", 13);
                tape.blinkMultiColor(COLOR_RED, COLOR_BLUE, 200);
            }
            selectOption(timer);
        }
        return true;
    }
    else
    {
        tape.blink(COLOR_RED, 200);
        return false;
    }
}

bool UserInterface::read()
{
    uint8_t data[4] = {};
    uint8_t flag = 0;
    
    while (1)
    {
        if (flag == 0b1111)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    status_[j+i*6] = data[i] & (1 << j);
                }
            }
            serialUITimeoutTimer_.reset();
            return true;
        }
        if (serialUI_.available() > 0)
        {
            uint8_t tmp = serialUI_.read();
            flag |= 1 << ((tmp&(0b11<<6))>>6);
            data[(tmp&(0b11<<6))>>6] = tmp;
        }
        if (serialUITimeoutTimer_.wait(200))
        {
            return false;
        }
    }
}

void UserInterface::debug()
{
    if (status_[MODE_COLLECTION] and not status_[MODE_TOWEL] and not status_[MODE_SHEET])
    {
        if (check_ != COLLECTION)
        {
            LCD.clearAll();
            check_ = COLLECTION;
        }
        collection.debug(status_);
        towel.stopDebug();
        sheet.stopDebug();
        chassis.stopDebug();
    }
    else if (not status_[MODE_COLLECTION] and status_[MODE_TOWEL] and not status_[MODE_SHEET])
    {
        if (check_ != TOWEL)
        {
            LCD.clearAll();
            check_ = TOWEL;
        }
        towel.debug(status_);
        collection.stopDebug();
        sheet.stopDebug();
        chassis.stopDebug();
    }
    else if (not status_[MODE_COLLECTION] and not status_[MODE_TOWEL] and status_[MODE_SHEET])
    {
        if (check_ != SHEET)
        {
            LCD.clearAll();
            check_ = SHEET;
        }
        sheet.debug(status_);
        collection.stopDebug();
        towel.stopDebug();
        chassis.stopDebug();
    }
    else
    {
        if (check_ != NONE)
        {
            LCD.clearAll();
            check_ = NONE;
        }
        odom.printData();
        printData();
        chassis.debug(status_);
        collection.stopDebug();
        towel.stopDebug();
        sheet.stopDebug();
    }
}

void UserInterface::selectMode()
{
    if (status_[MODE_COLLECTION])
    {
        LCD.clear(0x14, 9);
        tape.blink(not isTestingOutWork() ? COLOR_GREEN : COLOR_PINK, 250);
        odom.setYawBias(status_[ZONE] == ZONE_BLUE ? 90 : -90);
    }
    else if (status_[MODE_TOWEL])
    {
        LCD.clear(0x14, 9);
        tape.blink(not isTestingOutWork() ? COLOR_GREEN : COLOR_PINK, 250);
        odom.setYawBias(180);
        towel.initialPos_ = false;
    }
    else if (status_[MODE_SHEET])
    {
        LCD.clear(0x14, 9);
        tape.blink(not isTestingOutWork() ? COLOR_GREEN : COLOR_PINK, 250);
        odom.setYawBias(180);
    }
    else
    {
        LCD.print(0x14, "NO MODE", 9);
        tape.set(COLOR_BLUE);
    }
}

void UserInterface::selectOption(MyTimer& timer)
{
    if (TEST_RUN and not digitalRead(PIN_TEST_OUT))
    {
        if (not isPushed_)
        {
            isTestingOutWork_ = not isTestingOutWork_;
            LED.set(1, isTestingOutWork_);
            isPushed_ = true;
        }
    }
    else if (status_[RESET_ODOMETRY])
    {
        if (not isPushed_)
        {
            resetOdometry(timer);
            flush();
            isPushed_ = true;
        }
    }
    else if (status_[INITIALIZE_MECHANISM_POSITION])
    {
        if (not isPushed_)
        {
            LCD.print(0x00, "Initializing...", 16);
            LCD.print(0x10, "Release EMSSW", 16);
            tape.set(COLOR_BLUE);
            if (status_[MODE_COLLECTION])
            {
                collection.moveBackMechanismToInitialPosition();
                collection.reset();
            }
            if (status_[MODE_TOWEL])
            {
                towel.moveBackMechanismToInitialPosition();
                towel.reset();
            }
            if (status_[MODE_SHEET])
            {
                sheet.moveBackMechanismToInitialPosition();
                sheet.reset();
            }
            LCD.clearAll();
            flush();
            timer.reset();
            isPushed_ = true;
        }
    }
    else
    {
        isPushed_ = false;
    }
}

void UserInterface::resetOdometry(MyTimer& timer)
{
    LCD.clearAll();
    LCD.print(0x00, "Resetting Odom...");
    LCD.print(0x10, "DO NOT MOVE THIS");
    tape.set(COLOR_RED);
    odom.reset();
    timer.reset();
    while (not timer.wait(800));
    LCD.clearAll();
    hasResetOdometry_ = true;
}

void UserInterface::resetGyro(MyTimer& timer)
{
    LCD.clearAll();
    LCD.print(0x00, "Resetting Gyro...");
    LCD.print(0x10, "DO NOT MOVE THIS");
    tape.set(COLOR_RED);
    odom.resetGyro();
    timer.reset();
    while (not timer.wait(800));
    LCD.clearAll();
}

void UserInterface::monitorGyro(MyTimer& watchdogTimer)
{
    if (not odom.canReceiveGyroData())
    {
        if (watchdogTimer.wait(100))
        {
            resetGyro(watchdogTimer);
        }
    }
    else
    {
        watchdogTimer.reset();
    }
}

void UserInterface::reset()
{
    isPushed_ = false;
    isDebugging_ = false;
    hasResetOdometry_ = false;
    flush();
}

void UserInterface::printData()
{
    String str = status_[ROUND] == ROUND_PRELIMINARY ? "P" : "F";
    str += status_[ZONE] == ZONE_BLUE ? "B" : "R";
    str += status_[MODE_COLLECTION] ? "C:" : (status_[MODE_TOWEL] ? "T:" : (status_[MODE_SHEET] ? "S:" : "N:"));
    LCD.print(0x10, str);
}

bool UserInterface::startup()
{
    return not digitalRead(PIN_START1) or not digitalRead(PIN_START2);
}

bool UserInterface::escape()
{
    static bool isPushed = false;
    if (not digitalRead(PIN_RESET))
    {
        if (not isPushed)
        {
            isPushed = true;
            return true;
        }
    }
    else
    {
        isPushed = false;
    }
    
   return false;
}

void UserInterface::flush()
{
    while (serialUI_.available() > 0)
    {
        serialUI_.read();
    }
}

bool UserInterface::isDebugging() const
{
    return isDebugging_;
}

bool UserInterface::hasResetOdometry() const
{
    return hasResetOdometry_;
}

bool UserInterface::isDuringEmergencyStop() const
{
    return not digitalRead(PIN_EMS);
}

bool UserInterface::isForbidden() const
{
    return not digitalRead(PIN_EMS);
}

bool UserInterface::isReadyToWork() const
{
    return not isDebugging() and not isDuringEmergencyStop() and hasResetOdometry();
}

bool UserInterface::isTestingOutWork() const
{
    return isTestingOutWork_;
}

UserInterface ui;