#include "base.hpp"
#include "../common/chassis.hpp"
#include "../common/definition.hpp"
#include "../common/i2c_lcd.hpp"
#include "../common/odometry.hpp"
#include "../common/sbdbt.hpp"
#include "../common/tape_led.hpp"
#include "../common/onboard_led.hpp"
#include "user_interface.hpp"
#include "action.hpp"
#include "collection.hpp"
#include "towel.hpp"
#include "sheet.hpp"

Base::Base() : 
    generalPurposeTimer_(8)
{}

void Base::initialize()
{
    SerialUSB.begin(115200);
    LED.turnOff();
    tape.turnOff();
    ui.initialize();
    bt.initialize();
    LCD.initialize();
    odom.initialize();

    while (not generalPurposeTimer_.wait(200));
    Motor::initialize();
    Solenoid::initialize();
    collection.initialize();
    towel.initialize();
    sheet.initialize();
    chassis.brake();
    interrupts();
}

void Base::run()
{
    if (odom.refresh() and not ui.isDebugging()) 
    {
        odom.printData();
    }
    if (ui.escape()) 
    {
        quit();
    }
    
    if (isBusy_) work();  
    else         standby();
}

void Base::work()
{
    ui.printData();
    if (ui.isForbidden())
    {
        tape.turnOff();
        if (TEST_RUN) suspend();
        else          quit();
        return;
    }

    if (collection.work(generalPurposeTimer_)) exit();
    if (towel.work(generalPurposeTimer_))      exit();
    if (sheet.work(generalPurposeTimer_))      exit();
}

void Base::standby()
{
    LCD.enabled_ = true;
    bt.write(COM_CODE::RESUME);
    ui.monitor(generalPurposeTimer_);

    if (ui.isReadyToWork() and ui.startup())
    {
        getStarted();
    }
}

void Base::getStarted()
{
    bt.flush();
    LCD.clearAll();
    generalPurposeTimer_.reset();
    if (ui[MODE_COLLECTION]) isBusy_ = collection.getStarted();
    else if (ui[MODE_TOWEL]) isBusy_ = towel.getStarted();
    else if (ui[MODE_SHEET]) isBusy_ = sheet.getStarted();
}

void Base::exit()
{
    LCD.enabled_ = true;
    LCD.clearAll();
    bt.flush();
    ui.reset();
    generalPurposeTimer_.reset();
    collection.reset();
    towel.reset();
    sheet.reset();
    action.reset();
    isBusy_ = false;
}

void Base::suspend()
{
    LCD.print(0x10, "SUSP");
    chassis.brake();
    generalPurposeTimer_.reset();
    collection.stopTimeoutTimer();
}

void Base::quit()
{
    exit();
}

void Base::reset()
{
    exit();
}

Base ikaruga;