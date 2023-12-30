#include "sheet.hpp"
#include "../common/definition.hpp"
#include "../common/component/vector2.hpp"
#include "../common/i2c_lcd.hpp"
#include "../common/sbdbt.hpp"
#include "../common/chassis.hpp"
#include "../common/odometry.hpp"
#include "../common/tape_led.hpp"
#include "user_interface.hpp"
#include "action.hpp"
#include "towel.hpp"

Sheet::Sheet() :
    rogerEncorder_(3, 2),
    rogerMotor_(7),
    gripperSolenoid_(15),
    sliderSolenoid_(14),
    lifterSolenoid_(13),
    viaPoint_({
                {1633+200, 7300+300},
                {3267+200, 7300+300}
              }),
    pointHangOut_({
                    {1150+383+200+100, 6900+300},
                    {1150+2800-383-200+150, 6900+300}
                  }),
    routeSZ2ToMyFriend_({
                          {{0, 0}, {100+200, 0+300}, {200+200, 7450+300}, {3700+200, 7600+300}},
                          {{0, 0}, {100+200, 0+300}, {50+200, 5700+300}, {1800+200, 5700+300}, {1800+200, 7500+300}, {900+200, 7600+300}}
                        }),
    routeSheetToSZ2_({
                       {pointHangOut_[ZONE_BLUE]+DIST_MOVE_BACKWARD, {0+200, 7500+300}, {0+200, 0+300}},
                       {pointHangOut_[ZONE_RED]+DIST_MOVE_BACKWARD, {0+200, 7500+300}, {0+200, 0+300}}
                     }),
    routeTowelToMyFriend_({
                           {
                             { // ZONE_BLUE
                               {{1750+200, 5400+300}, {1750+200, 7450+300}, {3500+200, 7600+300}},
                               {{2965+200, 5400+300}, {2950+200, 7450+300}, {3500+200, 7600+300}},
                               {{2950+200, 3400+300}, {2950+200, 7450+300}, {3500+200, 7600+300}}
                             },
                             { // ZONE_RED
                               {{3200+200, 5400+300}, {2950+200, 7450+300}, {900+200, 7600+300}},
                               {{2125+200, 5400+300}, {1750+200, 7450+300}, {900+200, 7600+300}},
                               {{2125+200, 3400+300}, {2125+200, 7450+300}, {900+200, 7600+300}}
                             }
                           },
                           {
                             { // ZONE_BLUE
                               {{1890+200, 5400+300}, {1890+200, 7450+300}, {3500+200, 7600+300}},
                               {{3210+200, 5400+300}, {3210+200, 7450+300}, {3500+200, 7600+300}}
                             },
                             { // ZONE_RED
                               {{3250+200, 5400+300}, {3250+200, 7450+300}, {900+200, 7600+300}},
                               {{1890+200, 5400+300}, {1890+200, 7450+300}, {900+200, 7600+300}}
                             }
                           }
                         })
{}


void Sheet::initialize()
{
    pinMode(PIN_LIMIT_ELAVATE_BOTTOM, INPUT);
    pinMode(PIN_LIMIT_POLE_GUIDE, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_BACK_LEFT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_BACK_RIGHT, INPUT);
    reset();
}

bool Sheet::getStarted()
{
    zone_ = ui[ZONE];
    action.setYawTarget(45);
    bt.flush();
    actionFlag_ = GRIP;
    isLowering_ = true;
    if (not ui.isTestingOutWork())
    {
        processFlag_ = START;
    }
    else
    {
        odom.setYawBias(45);
        processFlag_ = APPROACH_MY_FRIEND;
    }
    return true;
}

bool Sheet::startFromTheMiddle(int towelIndex)
{
    towelIndex_ = towelIndex;
    zone_ = ui[ZONE];
    action.setYawTarget(45);
    bt.flush();
    isLowering_ = true;
    processFlag_ = START_FROM_TOWEL;
    return true;
}

bool Sheet::work(MyTimer& delayTimer)
{
    if (processFlag_ == STANDBY)
    {
        return false;
    }

    checkActionReguraly();

    if (processFlag_ == START)
    {
        LCD.print(0x14, "START", 9);
        if (action.follow(routeSZ2ToMyFriend_[zone_]))
        {
            expandSlider();
            liftHand();
            processFlag_ = APPROACH_MY_FRIEND;
        }
        isLowering_ = true;
    }
    else if (processFlag_ == START_FROM_TOWEL)
    {
        LCD.print(0x14, "START TWL", 9);
        if (bt.read() == SHEET_COM_CODE::READY_TO_HANG_OUT)
        {
            isReceived_ = true;
        }
        if (isReceived_)
        {
            if (action.follow(routeTowelToMyFriend_[ui[ROUND]][zone_][towelIndex_]))
            {
                expandSlider();
                liftHand();
                isLowering_ = false;
                processFlag_ = APPROACH_MY_FRIEND;
            }
        }
        else
        {
            tape.set(COLOR_GREEN);
            chassis.brake();
        }
    }
    else if (processFlag_ == APPROACH_MY_FRIEND)
    {
        LCD.print(0x14, "APPROACH", 9);
        if (not hasCorrected_)
        {
            tape.blink(COLOR_YELLOW, 200);
            if (not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT))
            {
                chassis.brake();
                hasCorrected_ = true;
                bt.flush();
            }
            else
            {
                action.move({0, 200});
            }
        }
        else
        {
            tape.blink(COLOR_BLUE, 200);
            if (bt.read() == SHEET_COM_CODE::GRIP_SHEET)
            {
                chassis.brake();
                hasCorrected_ = false;
                processFlag_ = PICKUP;
                return false;
            }
            chassis.drive({-300, not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT) ? 10 : 150}, 80*setSign(action.getYawDeviation(), 2));
        }
    }
    else if (processFlag_ == PICKUP)
    {
        LCD.print(0x14, "PICKUP", 9);
        tape.set(COLOR_MAGENTA);
        if (pickup(delayTimer))
        {
            bt.write(SHEET_COM_CODE::MOVED);
            action.setYawTarget(135);
            processFlag_ = LEAVE_MY_FRIEND;
        }
    }
    else if (processFlag_ == LEAVE_MY_FRIEND)
    {
        LCD.print(0x14, "LEAVE", 9);
        bt.write(SHEET_COM_CODE::MOVED);
        if (action.moveToward(viaPoint_[zone_], 600))
        {
            if (elevateRoger(COUNT_2000))
            {
                processFlag_ = APPROACH_POINT_HANG_OUT;
            }
            else
            {
                LCD.print(0x14, "ELEVATING", 9);
                tape.set(COLOR_BLUE);
            }
        }
        else
        {
            tape.blinkMultiColor(COLOR_YELLOW, COLOR_BLUE, 200);
        }
        isElevating_ = true;
    }
    else if (processFlag_ == APPROACH_POINT_HANG_OUT)
    {
        LCD.print(0x14, "APPROACH", 9);
        tape.set(COLOR_YELLOW);
        if (action.moveTo(pointHangOut_[zone_], 300))
        {
            processFlag_ = HANG_OUT;
        }
    }
    else if (processFlag_ == HANG_OUT)
    {
        LCD.print(0x14, "HANGOUT", 9);
        tape.set(COLOR_MAGENTA);
        if (hangOut(delayTimer))
        {
            processFlag_ = LEAVE_POINT_HANG_OUT;
        }
    }
    else if (processFlag_ == LEAVE_POINT_HANG_OUT)
    {
        LCD.print(0x14, "LEAVE", 9);
        if (action.moveToward({pointHangOut_[zone_]+DIST_MOVE_BACKWARD}))
        {
            tape.set(COLOR_BLUE);
            if (ui.isTestingOutWork())
            {
                if (lowerRoger())
                {
                    reset();
                    return true;
                }
            }
            else
            {
                shrinkSlider();
                liftHand();
                isLowering_ = true;
                isReceived_ = false;
                if (rogerEncorder_.getCount() <= 2500) 
                {
                    action.setYawTarget(180);
                    processFlag_ = RETURN;
                }
            }
        }
        else
        {
            tape.set(COLOR_YELLOW);
        }
    }
    else if (processFlag_ == RETURN)
    {
        LCD.print(0x14, "RETURN", 9);
        if (zone_ == ZONE_BLUE)
        {
            if (action.follow(routeSheetToSZ2_[zone_], Action::SEND))
            {
                reset();
                return true;
            }
        }
        else
        {
            if (bt.read() == COM_CODE::RESUME)
            {
                isReceived_ = true;
            }
            if (isReceived_)
            {
                if (action.follow(routeSheetToSZ2_[zone_]))
                {
                    reset();
                    return true;
                }
            }
            else
            {
                tape.set(COLOR_WHITE);
            }
        }
    }
    return false;
}

bool Sheet::pickup(MyTimer& delayTimer)
{
    if (actionFlag_ == GRIP)
    {
        if (delayTimer.wait(500))
        {
            odom.correct({zone_==ZONE_BLUE ? -5000+800+384-200 : 100+384+200, 8200-383+300});
            bt.write(SHEET_COM_CODE::GRIPED);
            actionFlag_ = PULL;
        }
        gripSheet();
    }
    else if (actionFlag_ == PULL)
    {
        bt.write(SHEET_COM_CODE::GRIPED);
        if (odom.getNow().x >= (zone_==ZONE_BLUE ? -5000+800+384-200 +200 : 100+384+200 +200))
        {
            actionFlag_ = PRESS;
            return true;
        }
        else
        {
            chassis.drive({700, 0}, 80*setSign(action.getYawDeviation(), 2));
        }
    }
    return false;
}

bool Sheet::hangOut(MyTimer& delayTimer)
{
    if (actionFlag_ == PRESS)
    {
        if (not digitalRead(PIN_LIMIT_POLE_GUIDE))
        {
            chassis.brake();
            bt.write(SHEET_COM_CODE::PRESS_SHEET);
            isReceived_ = false;
            actionFlag_ = DROP;
        }
        else
        {
            tape.blink(COLOR_BLUE, 150);
            action.move({0, -200});
        }
    }
    else if (actionFlag_ == DROP)
    {
        if (isReceived_)
        {
            if (not digitalRead(PIN_LIMIT_POLE_GUIDE))
            {
                chassis.brake();
                if (bt.read() == SHEET_COM_CODE::READY_TO_RELEASE)
                {
                    actionFlag_ = RELEASE;
                }
            }
            else
            {
                action.move({0, -100});
            }
            dropHand();
        }
        else
        {
            bt.write(SHEET_COM_CODE::PRESS_SHEET);
            if (bt.read() == SHEET_COM_CODE::HOOK_ON)
            {
                isReceived_ = true;
            }
        }
    }
    else if (actionFlag_ == RELEASE)
    {
        bt.write(SHEET_COM_CODE::READY_TO_RELEASE);
        if (delayTimer.wait(800))
        {
            isReceived_ = false;
            actionFlag_ = PULL;
        }
        releaseSheet();
    }
    else if (actionFlag_ == PULL)
    {
        if (not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_BACK_LEFT) or delayTimer.wait(5000))
        {
            chassis.brake();
            odom.correct({zone_ == ZONE_BLUE ? -1150-383-200 : 1150+2800-383+200, odom.getNow().y});
            return true;
        }
        else
        {
            chassis.drive({100, not digitalRead(PIN_LIMIT_POLE_GUIDE) ? -15 : -70}, 80*setSign(action.getYawDeviation(), 2));
        }
    }
    return false;
}

void Sheet::checkActionReguraly()
{
    if (isElevating_)
    {
        if (elevateRoger(COUNT_2000))
        {
            isElevating_ = false;
        }
    }
    if (isLowering_)
    {
        if (lowerRoger())
        {
            isLowering_ = false;
        }
    }
}

bool Sheet::elevateRoger(long count)
{
    long devCount = count - rogerEncorder_.getCount();
    if (devCount >= -50 and devCount <= 50)
    {
        rogerMotor_.setPWM(0);
        return true;
    }
    else
    {
        rogerMotor_.setPWM(constrain(fabs(devCount*KP_ELEVATE), 30, 100)*setSign(-devCount));
        return false;
    }
}

bool Sheet::lowerRoger()
{
    if (not digitalRead(PIN_LIMIT_ELAVATE_BOTTOM))
    {
        rogerMotor_.setPWM(0);
        rogerEncorder_.reset();
        return true;
    }
    else
    {
        rogerMotor_.setPWM(40);
        return false;
    }
}

void Sheet::stopElevateAndLower()
{
    rogerMotor_.setPWM(0);
}

void Sheet::expandSlider()
{
    sliderSolenoid_.open();
}

void Sheet::shrinkSlider()
{
    sliderSolenoid_.close();
}

void Sheet::gripSheet()
{
    gripperSolenoid_.open();
}

void Sheet::releaseSheet()
{
    gripperSolenoid_.close();
}

void Sheet::liftHand()
{
    lifterSolenoid_.open();
}

void Sheet::dropHand()
{
    lifterSolenoid_.close();
}

void Sheet::moveBackMechanismToInitialPosition()
{
    while (1)
    {
        if (lowerRoger())
        {
            return;
        }
        if (ui.escape())
        {
            stopElevateAndLower();
            return;
        }
    }
}

void Sheet::checkSensorStatus()
{
    LCD.print(0x00, rogerEncorder_.getCount(), 5);
    LCD.print(0x10, digitalRead(PIN_LIMIT_ELAVATE_BOTTOM));
    LCD.print(0x12, digitalRead(PIN_LIMIT_POLE_GUIDE));
    LCD.print(0x14, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_BACK_LEFT));
    LCD.print(0x16, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT));
    LCD.print(0x18, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_BACK_RIGHT));
}

void Sheet::debug(bool(&status)[24])
{
    checkSensorStatus();

    if (status[MOTOR3_FORWARD])      elevateRoger(COUNT_2000);
    else if (status[MOTOR3_REVERSE]) lowerRoger();
    else                             stopElevateAndLower();

    if (status[TOGGLE0])             gripSheet();
    else                             releaseSheet();

    if (status[TOGGLE1])             expandSlider();
    else                             shrinkSlider();

    if (status[TOGGLE2])             liftHand();
    else                             dropHand();
}

void Sheet::stopDebug()
{
    stopElevateAndLower();
    releaseSheet();
    shrinkSlider();
    liftHand();
}

void Sheet::reset()
{
    processFlag_ = STANDBY;
    towelIndex_ = 0;
    hasCorrected_ = false;
    isReceived_ = false;
    isElevating_ = false;
    isLowering_ = false;
    rogerEncorder_.reset();
    rogerMotor_.brake();
    shrinkSlider();
    releaseSheet();
    liftHand();
}

Sheet sheet;