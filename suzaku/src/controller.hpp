#pragma once

#include <Arduino.h>
#include "component/my_timer.hpp"

#define ZONE_RED            (controller.isZoneRed())
#define ZONE_BLUE           (!ZONE_RED)
#define ZONE                (ZONE_RED ? 0 : 1)
#define FINAL               (controller.isFinal())
#define DUET                (controller.isDuet())
#define SOLO                (!DUET)
#define FROM_SZ             (controller.startFromSZ())
#define MODE_COLLECTION     (controller.getMode()==Controller::MODE::COLLECTION)
#define MODE_TSHIRT         (controller.getMode()==Controller::MODE::TSHIRT)
#define MODE_TOWEL          (controller.getMode()==Controller::MODE::TOWEL)
#define MODE_TSHIRT_N_TOWEL (controller.getMode()==Controller::MODE::TSHIRT_N_TOWEL)
#define MODE_SHEET          (controller.getMode()==Controller::MODE::SHEET)
#define MODE_NONE           (controller.getMode()==Controller::MODE::NONE)

class Controller
{   
    friend void ISRReset();

public:

    enum MODE
    {
        COLLECTION,
        TSHIRT,
        TOWEL,
        TSHIRT_N_TOWEL,
        SHEET,
        NONE
    };

    Controller();

    void initialize();

    void monitor();

    void debug();

    void flush();

    MODE getMode();

    bool isReady();

    bool isDebugging();
    
    bool isZoneRed();

    bool isFinal();

    bool isDuet();

    bool startFromSZ();

private:

    bool read();

    bool referTo(int, int);

    USARTClass& serialController_;

    MyTimer timeoutTimer_;
    
    DueTimer resetTimer_;

    MODE mode_ = MODE::NONE;

    String modeName_[6] = {"COLLECT", "TSHIRT", "TOWEL", "TSHIRT&TOWEL", "SHEET", ""};

    byte status_[4] = {};

    bool isDebugging_, isZoneRed_, isFinal_, isDuet_, startFromSZ_;
    
    bool isOdometryReset_ = false;

    volatile bool isResettingGyro_ = false;
};

extern Controller controller;