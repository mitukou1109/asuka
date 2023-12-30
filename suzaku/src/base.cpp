#include "base.hpp"
#include <Arduino.h>
#include "action.hpp"
#include "chassis.hpp"
#include "controller.hpp"
#include "i2c_lcd.hpp"
#include "odometry.hpp"
#include "sbdbt.hpp"
#include "tape_led.hpp"
#include "movement/collection.hpp"
#include "movement/sheet.hpp"
#include "movement/tshirt.hpp"
#include "movement/towel.hpp"

Base::Base() :
timer_(8)
{}

void Base::work()
{
    if(MODE_COLLECTION)
    {
        if(collection.work()) exit();
    }
    else if(MODE_TSHIRT)
    {
        if(tshirt.work()) exit();
    }
    else if(MODE_TOWEL)
    {
        if(towel.work()) exit();
    }
    else if(MODE_TSHIRT_N_TOWEL)
    {
        if(!isTshirtDone_)
        {
            if(tshirt.work()) isTshirtDone_ = true;
        }
        else
        {
            if(towel.work()) exit();
        }
    }
    else if(MODE_SHEET)
    {
        if(sheet.work()) exit();
    }
}

void Base::test()
{
    if(MODE_COLLECTION)
    {
        if(collection.test()) exit();
    }
    else if(MODE_TSHIRT)
    {
        if(tshirt.test()) exit();
    }
    else if(MODE_TOWEL)
    {
        if(towel.test()) exit();
    }
    else if(MODE_SHEET)
    {
        if(sheet.test()) exit();
    }
}

void Base::run()
{
    if(odom.refresh())
    {
        if(isBusy() || !controller.isDebugging()) odom.printData();
    }

    if(isBusy())
    {
        if(!digitalRead(PIN_ESCAPE)) exit();
        else if(isAtWork_)           work();
        else if(isOnTest_)           test();
    }
    else
    {
        controller.monitor();
        bt.transmit(6);

        setLED(0, isStarterPressed());
        setLED(1, !digitalRead(PIN_ESCAPE));

        if(tape.isSWReleased())
        {
            if(isStarterPressed())
            {
                if(controller.isReady())
                {
                    isAtWork_ = true;
                    start();
                }
                else if(controller.isDebugging() && !(MODE_NONE||MODE_TSHIRT_N_TOWEL))
                {
                    if(timer_.wait(1000))
                    {
                        isOnTest_ = true;
                        start();
                        lcd.print(0x1c, "TEST");
                    }
                }
            }
            else
            {
                timer_.reset();
            }
        }
    }
}

void Base::initialize()
{
    pinMode(PIN_ESCAPE, INPUT);
    for(int i=0;i<4;i++) pinMode(PIN_LED_ONBOARD[i], OUTPUT);
    tape.initialize();
    lcd.initialize();
    controller.initialize();
    bt.initialize();
    odom.initialize();
    collection.initialize();
    tshirt.initialize();
    towel.initialize();
    sheet.initialize();

    reset();
}

void Base::reset()
{
    chassis.brake();
    action.reset();
    collection.reset();
    sheet.reset();
    towel.reset();
    tshirt.reset();
    timer_.reset();
    controller.flush();
    lcd.clearLine(0x10);
    for(int i=0;i<4;i++) setLED(i, LOW);
}

void Base::start()
{
    reset();

    if(MODE_COLLECTION)            collection.start();
    else if(MODE_SHEET)            sheet.start();
    else if(MODE_TSHIRT)           tshirt.start();
    else if(MODE_TOWEL)            towel.start();
    else if(MODE_TSHIRT_N_TOWEL) { tshirt.start(); towel.start(); }
}

void Base::exit()
{
    reset();

    isAtWork_ = false;
    isOnTest_ = false;
    isTshirtDone_ = false;
}

void Base::setLED(int index, bool state)
{
    digitalWrite(PIN_LED_ONBOARD[index], state);
}

bool Base::isStarterPressed()
{
    return !digitalRead(PIN_STARTER_1) || !digitalRead(PIN_STARTER_2);
}

bool Base::isBusy()
{
    return isAtWork_ || isOnTest_;
}

bool Base::isOnTest()
{
    return isOnTest_;
}

Base base;