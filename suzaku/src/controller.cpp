#include "controller.hpp"
#include "base.hpp"
#include "chassis.hpp"
#include "i2c_lcd.hpp"
#include "odometry.hpp"
#include "tape_led.hpp"
#include "movement/collection.hpp"
#include "movement/sheet.hpp"
#include "movement/tshirt.hpp"
#include "movement/towel.hpp"

#define TACT_RESET (referTo(0,0))
#define TACT_SHIFT (referTo(0,1))
#define TGGL_RED   (referTo(0,2))
#define TGGL_FINAL (referTo(0,3))
#define TGGL_0     (referTo(0,4))
#define TGGL_1     (referTo(0,5))
#define TGGL_2     (referTo(1,0))
#define TGGL_3     (referTo(1,1))
#define TGGL_4     (referTo(1,2))
#define TGGL_DEBUG (referTo(1,3))
#define TGGL_DUET  (referTo(1,4))
#define TGGL_SZ    (referTo(1,5))
#define TGGL_M0    (referTo(2,0))
#define TGGL_M1    (referTo(2,1))
#define TGGL_M2    (referTo(2,2))
#define TACT_3A    (referTo(2,3))
#define TACT_3B    (referTo(2,4))
#define TACT_2A    (referTo(2,5))
#define TACT_2B    (referTo(3,0))
#define TACT_1A    (referTo(3,1))
#define TACT_1B    (referTo(3,2))
#define TACT_0A    (referTo(3,3))
#define TACT_0B    (referTo(3,4))

void ISRReset()
{
    controller.resetTimer_.stop();
    controller.isResettingGyro_ = false;
}

Controller::Controller() :
serialController_(Serial1),
resetTimer_(4),
timeoutTimer_(5)
{}

void Controller::initialize()
{
    serialController_.begin(115200);
    resetTimer_.attachInterrupt(ISRReset).setPeriod(1000000);
}

void Controller::monitor()
{
    if(read())
    {
        timeoutTimer_.reset();
        base.setLED(3, LOW);
    }
    else
    {
        if(timeoutTimer_.wait(500))
        {
            base.reset();
            base.setLED(3, HIGH);
        }
        return;
    }

    if(TACT_RESET)
    {
        odom.reset();
        isOdometryReset_ = true;

        if(TACT_SHIFT)
        {
            if(!isResettingGyro_)
            {
                odom.resetGyro();
                resetTimer_.start();
                isResettingGyro_ = true;
            }
        }
    }

    odom.setYawAdd(((MODE_COLLECTION && !FROM_SZ) || MODE_NONE) ? 0 : 180);
    
    if(isDebugging())
    {
        tape.set(COLOR_CYAN);
        debug();
    }
    else
    {
        lcd.print(0x10, modeName_[mode_], 13);
        lcd.print(0x1d, isResettingGyro_ ? "RST" : "", 3);
        if(isResettingGyro_)
        {
            tape.set(COLOR_RED);
        }
        else
        {
            if(isOdometryReset_)
            {
                if(mode_==MODE::NONE) tape.blink(COLOR_GREEN, COLOR_BLUE);
                else tape.blink(COLOR_GREEN);
            }
            else
            {
                tape.blink(COLOR_RED, COLOR_BLUE);
            }
        }
    }
}

void Controller::debug()
{
    lcd.print(0x1c, startFromSZ_);
    lcd.print(0x1d, isZoneRed_);
    lcd.print(0x1e, isFinal_);
    lcd.print(0x1f, isDuet_);

    collection.debug();
    sheet.debug();
    tshirt.debug();
    towel.debug();
    
    if(TACT_SHIFT)
    {
        if(TACT_0A)      chassis.drive({0,0}, 200);
        else if(TACT_0B) chassis.drive({0,0}, -200);
        else             chassis.brake();

        if(TACT_1A)      sheet.extendSlider();
        else if(TACT_1B) sheet.shrinkSlider();
        else             sheet.stopSlider();

        if(TACT_2A)      sheet.adjustRoger(Sheet::COUNT_ROGER_UNFOLD);
        else if(TACT_2B) sheet.lowerRoger();
        else             sheet.stopRoger();

        if(TACT_3A)      tshirt.adjustWing(Tshirt::COUNT_WING_RELEASE);
        else if(TACT_3B) tshirt.lowerWing();
        else             tshirt.stopWing();

        if(TGGL_0)       sheet.releaseDeliverySheet();
        else             sheet.pinchDeliverySheet();

        if(TGGL_1)       sheet.shrinkHook();
        else             sheet.freeHook();

        if(TGGL_2)       sheet.releaseSheet();
        else             sheet.freeSheet();

        if(TGGL_3)       tshirt.releaseHanger();
        else             tshirt.freeHanger();

        if(TGGL_4)       { towel.openFlipper(); towel.releaseTowel(); }
        else             { towel.foldFlipper(); towel.freeTowel(); }

        collection.stopConveyor();
        collection.stopArm();
        collection.openGate();
    }
    else
    {
        if(TACT_0A)      collection.driveConveyor(DIR_CCW);
        else if(TACT_0B) collection.driveConveyor(DIR_CW);
        else             collection.stopConveyor();

        if(TACT_1A)      collection.extendArm();
        else if(TACT_1B) collection.shrinkArm();
        else             collection.stopArm();

        if(TACT_2A)      sheet.adjustRoger(Sheet::COUNT_ROGER_HOOK);
        else if(TACT_2B) sheet.lowerRoger();
        else             sheet.stopRoger();

        if(TACT_3A)      tshirt.adjustWing(Tshirt::COUNT_WING_HOOK);
        else if(TACT_3B) tshirt.lowerWing();
        else             tshirt.stopWing();

        if(TGGL_0)       collection.closeGate();
        else             collection.openGate();

        if(TGGL_1)       sheet.extendHook();
        else             sheet.freeHook();

        if(TGGL_2)       sheet.gripSheet();
        else             sheet.freeSheet();

        if(TGGL_3)       tshirt.gripHanger();
        else             tshirt.freeHanger();

        if(TGGL_4)       { towel.openFlipper(); towel.gripTowel(); }
        else             { towel.foldFlipper(); towel.freeTowel(); }

        chassis.brake();
        sheet.stopSlider();
        sheet.pinchDeliverySheet();
    }
}

bool Controller::read()
{
    static byte flag = 0;

    if(serialController_.available()>0)
    {
        byte data = serialController_.read();
        int id = (data&(0b11<<6))>>6;
        status_[id] = data;
        flag |= 1<<id;

        if(flag == 0b1111)
        {
            flag = 0;

            isDebugging_ = TGGL_DEBUG;
            isZoneRed_ = TGGL_RED;
            isFinal_ = TGGL_FINAL;
            isDuet_ = TGGL_DUET;
            startFromSZ_ = TGGL_SZ;
            if     (TGGL_M0  && TGGL_M1  && TGGL_M2)  mode_ = MODE::COLLECTION;
            else if(TGGL_M0  && !TGGL_M1 && !TGGL_M2) mode_ = MODE::TSHIRT;
            else if(!TGGL_M0 && TGGL_M1  && !TGGL_M2) mode_ = MODE::TOWEL;
            else if(TGGL_M0 && TGGL_M1  && !TGGL_M2)  mode_ = MODE::TSHIRT_N_TOWEL;
            else if(!TGGL_M0 && !TGGL_M1 && TGGL_M2)  mode_ = MODE::SHEET;
            else                                      mode_ = MODE::NONE;
            
            return true;
        }
    }

    return false;
}

void Controller::flush()
{
    while(serialController_.available()>0) serialController_.read();
    timeoutTimer_.reset();
    isOdometryReset_ = false;
}

bool Controller::referTo(int id, int bit)
{
    return status_[id]&(1<<bit);
}

Controller::MODE Controller::getMode()
{
    return mode_;
}

bool Controller::isReady()
{
    return (mode_!=MODE::NONE) && !isDebugging_ && isOdometryReset_ && !isResettingGyro_; 
}

bool Controller::isDebugging()
{
    return isDebugging_;
}

bool Controller::isZoneRed()
{
    return isZoneRed_;
}

bool Controller::isFinal()
{
    return isFinal_;
}

bool Controller::isDuet()
{
    return isDuet_;
}

bool Controller::startFromSZ()
{
    return startFromSZ_;
}

Controller controller;