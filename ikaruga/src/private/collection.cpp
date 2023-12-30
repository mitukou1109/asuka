#include <algorithm>
#include "collection.hpp"
#include "../common/chassis.hpp"
#include "../common/definition.hpp"
#include "../common/odometry.hpp"
#include "../common/component/vector2.hpp"
#include "../common/i2c_lcd.hpp"
#include "../common/tape_led.hpp"
#include "../common/sbdbt.hpp"
#include "user_interface.hpp"
#include "action.hpp"
#include "towel.hpp"

Collection::Collection() :
    serialThinkpad_(Serial),
    thinkpadTimeoutTimer_(5),
    timeoutTimer_(6),
    extenderEncorder_(7, 6),
    extenderMotor_(4),
    extruderRelayMotor_(44, 45),
    catcherSolenoid_(2, 3),
    stretcherSolenoid_(0, 1),
    lifterSolenoid_(4),
    openerSolenoid_(5),
    pusherSolenoid_(6),
    routeSZ2ToTable_({
                       {{0+300, 0+200}, {4500+300, 1150+200}},
                       {{0+300, 0+200}, {4500+300, 0}}
                     }),
    routeTableToSZ2_({
                       {
                         {{4500+300, -100}, {0+300, -50+200}},
                         {{4500+300, 750}, {0+300, -50+200}}
                       },
                       {
                         {{4500+300, 1200+200}, {4500+300, 500}, {0+300, -50+200}},
                         {{4500+300, 500}, {0+300, -50+200}}
                       }
                     })
{}

void raiseTimeoutFlag()
{
    if (collection.seconds_/10 >= collection.TIME_LIMIT)
    {
        collection.seconds_ = 0;
        collection.timeoutFlag_ = true;
    }
    else
    {
        ++collection.seconds_;
    }
}

void Collection::initialize()
{
    serialThinkpad_.begin(115200);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT, INPUT);
    pinMode(PIN_LIMIT_EXTRUDE, INPUT);
    pinMode(PIN_LIMIT_STORE, INPUT);
    pinMode(PIN_LIMIT_ARM_EXTEND, INPUT);
    pinMode(PIN_LIMIT_ARM_SHORTEN, INPUT);
    pinMode(PIN_SERIAL_THINKPAD_RTS, OUTPUT);
    pinMode(PIN_UV_LED, OUTPUT);
    withdrawARequestToSend();
    reset();
    timeoutTimer_.attachInterrupt(raiseTimeoutFlag).setPeriod(100*1000);
}

bool Collection::getStarted()
{
    zone_ = ui[ZONE];
    side_ = ui[COLLECTION_SIDE];
    detectionRange_ = ui[COLLECTION_HALF_RANGE];
    isSameColor_ = ui[COLLECTION_SAME_COLOR];
    isOmeletteRice_ = ui[COLLECTION_OMELETTE_RICE];
    towel.initialPos_ = false;
    
    if (ui[MODE_TOWEL]) 
    {
        odom.setYawBias(180);
        action.setYawTarget(180);
        notCollecting_ = false;
        actionFlag_ = LIFT_GATE;
        processFlag_ = DISCHARGE;
    }
    else
    {
        action.setYawTarget(zone_==ZONE_BLUE ? 90 : -90);
        if (not ui.isTestingOutWork())
        {
            liftGate();
            processFlag_ = START;
        }
        else
        {
            odom.correct({zone_==ZONE_BLUE ? -4500-300 : 4500+300, side_==SIDE_BACK ? 1000+200 : 0});
            isElevating_ = true;
            processFlag_ = DETECT;
        }
    }
    return true;
}

bool Collection::work(MyTimer& delayTimer)
{
    if (processFlag_ == STANDBY)
    {
        return false;
    }

    checkActionRegularly();
    
    if (processFlag_ == START)
    {
        LCD.print(0x14, "START", 9);
        if (not action.isPaused())
        {
            if (delayTimer.wait(500))
            {
                closeGate();
                isElevating_ = true;
                if (isOmeletteRice_) serialThinkpad_.write("o\n");
                else                 serialThinkpad_.write("n\n");
            }
        }
        if (action.follow(routeSZ2ToTable_[side_], Action::MUTUAL))
        {
            delayTimer.reset();
            processFlag_ = DETECT;
        }
    }
    else if (processFlag_ == DETECT)
    {
        if (hasSetHeight_)
        {
            if (not hasCorrected_)
            {
                tape.blink(COLOR_YELLOW, 200);
                if (not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE))
                {
                    chassis.brake();
                    hasCorrected_ = true;
                }
                else
                {
                    shortenArm();
                    storeTowel();
                    dropGate();
                    releaseTowel();
                    withdrawCatcher();
                    hasExtended_ = false;
                    action.move({200, 0});
                }
            }
            else
            {
                if (timeoutTimerIsStopped_ and processFlag_ != REMOVE)
                {
                    timeoutTimer_.start();
                    timeoutTimerIsStopped_ = false;
                }

                if (timeoutFlag_ and processFlag_ != COLLECT)
                {
                    timeoutFlag_ = false;
                    somethingWentWrong_ = true;
                    chassis.brake();
                    processFlag_ = PREPARE_TO_RETURN;
                }

                if (side_ == SIDE_BACK)
                {
                    if (detectionRange_ != HALFWAY and not digitalRead(zone_==ZONE_BLUE ? PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT : PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT))
                    {
                        chassis.brake();
                        processFlag_ = PREPARE_TO_RETURN;
                        return false;
                    }
                    else if (detectionRange_ == HALFWAY and odom.getNow().y <= 800)
                    {
                        chassis.brake();
                        processFlag_ = PREPARE_TO_RETURN;
                        return false;
                    }
                }
                else 
                {
                    if (detectionRange_ != HALFWAY and odom.getNow().y >= 1400)
                    {
                        chassis.brake();
                        processFlag_ = PREPARE_TO_RETURN;
                        return false;
                    }
                    else if (detectionRange_ == HALFWAY and odom.getNow().y >= 500)
                    {
                        chassis.brake();
                        processFlag_ = PREPARE_TO_RETURN;
                        return false;
                    }
                }

                shortenArm();
                storeTowel();
                TOWEL_STATUS state = getTowelPosition();
                if (state == TOWEL_STATUS::MOVE)
                {
                    LCD.print(0x14, "MOVE");
                    LCD.print(0x18, position_, 5);
                    tape.set(COLOR_MAGENTA);
                    collectFlagTimes_ = 0;
                    if (position_ == 0)
                    {
                        chassis.brake();
                    }
                    else
                    {
                        double vX = not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE) ? 10 : 100;
                        double vY = (abs(position_)<=150 ? 80 : (abs(position_)<=250 ? 150 : 200))*setSign(position_);
                        if (zone_ == ZONE_BLUE)
                        {
                            vX *= -1;
                            vY *= -1;
                        }
                        chassis.drive({vX, vY}, 30*setSign(action.getYawDeviation(), 2));
                    }
                }
                else if (state == TOWEL_STATUS::REMOVE)
                {
                    if (removeFlagTimes_ >= 3)
                    {
                        chassis.brake();
                        processFlag_ = PREPARE_TO_RETURN;
                        return false;
                    }
                    else
                    {
                        withdrawARequestToSend();
                        stopTimeoutTimer();
                        collectFlagTimes_ = 0;
                        ++removeFlagTimes_;
                        hasCorrected_ = true;
                        chassis.brake();
                        actionFlag_ = WITHDRAW;
                        processFlag_ = REMOVE;
                    }
                }
                else if (state == TOWEL_STATUS::PULL)
                {
                    chassis.brake();
                    if (collectFlagTimes_ >= 3)
                    {
                        withdrawARequestToSend();
                        collectFlagTimes_ = 0;
                        actionFlag_ = SHORTEN;
                        notCollecting_ = false;
                        modePull_ = true;
                        processFlag_ = COLLECT;
                    }
                    else
                    {
                        ++collectFlagTimes_;
                    }                    
                }
                else if (state == TOWEL_STATUS::CATCH)
                {
                    chassis.brake();
                    if (collectFlagTimes_ >= 3)
                    {
                        withdrawARequestToSend();
                        collectFlagTimes_ = 0;
                        actionFlag_ = CATCH;
                        modePull_ = false;
                        notCollecting_ = false;
                        processFlag_ = COLLECT;
                    }
                    else
                    {
                        ++collectFlagTimes_;
                    }
                }
                else if (state == TOWEL_STATUS::ERR_NOTOWEL)
                {
                    LCD.print(0x14, "NOTOWEL", 9);
                    tape.set(COLOR_CYAN);
                    double vX = not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE) ? 10 : 200;
                    if (zone_ == ZONE_BLUE) vX *= -1;
                    chassis.drive({vX, side_ == SIDE_BACK ? -300 : 300}, 30*setSign(action.getYawDeviation(), 2));
                }
                else
                {
                    LCD.print(0x14, "TIMEOUT", 9);
                    tape.blink(COLOR_RED, 200);
                    double vX = not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE) ? 10 : 200;
                    if (zone_ == ZONE_BLUE) vX *= -1;
                    chassis.drive({vX, side_ == SIDE_BACK ? -300 : 300}, 30*setSign(action.getYawDeviation(), 2));
                }
            }
        }
    }
    else if (processFlag_ == REMOVE)
    {
        LCD.print(0x14, "REMOVE", 9);
        tape.blinkMultiColor(COLOR_RED, COLOR_WHITE, 50);
        if (not hasExtended_)
        {
            if (extendArm(position_))
            {
                if (delayTimer.wait(500))
                {
                    originalPoint_ = odom.getNow().y;
                    if (side_ == SIDE_BACK)
                    {
                        point_ = odom.getNow().y+600;
                        if (point_ > 1100) point_ = 1100;
                    }
                    else
                    {
                        point_ = odom.getNow().y-600;
                        if (point_ < 150) point_ = 150;
                    }

                    actionFlag_ = CATCH;
                    hasExtended_ = true;
                }
                stretchCatcher();
            }
        }
        else
        {
            if (actionFlag_ == CATCH)
            {
                if (delayTimer.wait(500))
                {
                    actionFlag_ = WITHDRAW;
                }
                catchTowel();
            }
            if (actionFlag_ == WITHDRAW)
            {
                if (delayTimer.wait(500))
                {
                    actionFlag_ = RELEASE;
                }
                withdrawCatcher();
            }
            else if (actionFlag_ == RELEASE)
            {
                if (fabs(point_ - odom.getNow().y) <= 30)
                {
                    chassis.brake();
                    if (extendArm(850))
                    {
                        if (delayTimer.wait(500))
                        {
                            actionFlag_ = SHORTEN;
                        }
                        releaseTowel();
                    }
                }
                else
                {
                    double vX = not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE) ? 10 : 100;
                    if (zone_ == ZONE_BLUE) vX *= -1;
                    chassis.drive({vX, constrain(fabs(point_ - odom.getNow().y)*KP_MOVE, 150, 600)*setSign(point_ - odom.getNow().y, 20)}, 70*setSign(action.getYawDeviation(), 2));
                }
            }
            else if (actionFlag_ == SHORTEN)
            {
                if (shortenArm())
                {
                    if (fabs(originalPoint_ - odom.getNow().y) <= 30)
                    {
                        chassis.brake();
                        hasExtended_ = false;
                        actionFlag_ = NONE;
                        clearSerialBuffer(serialThinkpad_);
                        processFlag_ = DETECT;
                    }
                    else
                    {
                        double vX = not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE) ? 10 : 100;
                        if (zone_ == ZONE_BLUE) vX *= -1;
                        chassis.drive({vX, constrain(fabs(originalPoint_ - odom.getNow().y)*KP_MOVE, 150, 600)*setSign(originalPoint_ - odom.getNow().y, 20)}, 70*setSign(action.getYawDeviation(), 2));
                    }
                }
            }
        }
    }
    else if (processFlag_ == COLLECT)
    {
        if (actionFlag_ == CATCH)
        {
            LCD.print(0x14, "COLLECT:C", 9);
        }
        else if (actionFlag_ == SHORTEN)
        {
            LCD.print(0x14, "COLLECT:P", 9);
        }
        tape.blink(COLOR_MAGENTA, 200);

        if (not hasExtended_)
        {
            if (extendArm(isSameColor_ ? position_-30 : position_))
            {
                if (delayTimer.wait(500))
                {
                    hasWithdrawn_ = false;
                    hasExtended_ = true;
                }
                stretchCatcher();
            }
        }
        else
        {
            if (actionFlag_ == CATCH)
            {
                if (isSameColor_)
                {
                    if (not hasExamined_)
                    {
                        isMyTowel_ = examineWhetherThisTowelIsMine();
                        hasExamined_ = true;
                    }
                    if (isMyTowel_)
                    {
                        if (delayTimer.wait(500))
                        {
                            catched_ = true;
                            isMyTowel_ = true;
                            actionFlag_ = WITHDRAW;
                        }
                        catchTowel();
                        liftGate();
                    }
                    else
                    {
                        actionFlag_ = WITHDRAW;
                    }
                }
                else
                {
                    if (delayTimer.wait(500))
                    {
                        catched_ = true;
                        actionFlag_ = WITHDRAW;
                    }
                    catchTowel();
                    liftGate();
                }
            }
            else if (actionFlag_ == WITHDRAW)
            {
                if (delayTimer.wait(500))
                {
                    hasWithdrawn_ = true;
                    actionFlag_ = SHORTEN;
                }
                withdrawCatcher();
            }
            else if (actionFlag_ == SHORTEN)
            {
                if (isSameColor_)
                {
                    if (not hasExamined_)
                    {
                        isMyTowel_ = examineWhetherThisTowelIsMine();
                        hasExamined_ = true;
                    }
                    if (isMyTowel_)
                    {
                        if (shortenArm())
                        {
                            actionFlag_ = RELEASE;
                        }
                    }
                    else
                    {
                        if (not hasWithdrawn_)
                        {
                            if (delayTimer.wait(500))
                            {
                                hasWithdrawn_ = true;
                            }
                            withdrawCatcher();
                            releaseTowel();
                        }
                        else
                        {
                            if (shortenArm())
                            {
                                if (side_ == SIDE_BACK)
                                {
                                    point_ = odom.getNow().y - 300;
                                }
                                else
                                {
                                    point_ = odom.getNow().y + 300;
                                }
                                positionPrev_ = odom.getNow().y;
                                actionFlag_ = MOVE;
                            }
                        }
                    }
                }
                else
                {
                    if (modePull_)
                    {
                        if (delayTimer.wait(100))
                        {
                            modePull_ = false;
                        }
                        catchTowel();
                    }
                    else
                    {
                        if (shortenArm())
                        {
                            actionFlag_ = RELEASE;
                        }
                    }
                }
            }
            else if (actionFlag_ == RELEASE)
            {
                if (delayTimer.wait(500))
                {
                    hasExtended_ = false;
                    catched_ = false;
                    hasExamined_ = false;
                    actionFlag_ = NONE;
                    clearSerialBuffer(serialThinkpad_);
                    positionPrev_ = odom.getNow().y;
                    processFlag_ = DETECT;
                }
                releaseTowel();
                withdrawCatcher();
                dropGate();
            }
            else if (actionFlag_ == MOVE)
            {
                if (fabs(point_ - odom.getNow().y) <= 30 or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE))
                {
                    chassis.brake();
                    hasExtended_ = false;
                    hasExamined_ = false;
                    actionFlag_ = NONE;
                    clearSerialBuffer(serialThinkpad_);
                    processFlag_ = DETECT;
                }
                else
                {
                    double vX = not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE) ? 10 : 150;
                    if (zone_ == ZONE_BLUE) vX *= -1;
                    chassis.drive({vX, constrain(fabs(point_ - odom.getNow().y)*KP_MOVE, 150, 600)*setSign(point_ - odom.getNow().y, 20)}, 70*setSign(action.getYawDeviation(), 2));
                }
            }
        }
    }
    else if (processFlag_ == PREPARE_TO_RETURN)
    {
        if (ui.isTestingOutWork())
        {
            reset();
            return true;
        }
        LCD.print(0x14, "PRE.RET.");
        tape.set(COLOR_BLUE);
        chassis.brake();
        withdrawARequestToSend();
        stopTimeoutTimer();
        if (not notCollecting_) action.setYawTarget(180);
        processFlag_ = RETURN;
    }
    else if (processFlag_ == RETURN)
    {
        LCD.print(0x14, "RETURN", 9);
        if (action.follow(routeTableToSZ2_[side_][detectionRange_], Action::MUTUAL))
        {
            actionFlag_ = LIFT_GATE;
            processFlag_ = notCollecting_ ? END : DISCHARGE;
        }
        if (not action.isPaused())
        {
            if (extendArm(350))
            {
                isLowering_ = true;
            }
        }
    }
    else if (processFlag_ == DISCHARGE)
    {
        if (not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE) or not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE))
        {
            chassis.brake();
            odom.correct({odom.getNow().x, 0+300});
            if (actionFlag_ == LIFT_GATE)
            {
                LCD.print(0x14, "LIFT GATE", 9);
                tape.set(COLOR_BLUE);
                if (delayTimer.wait(500))
                {
                    actionFlag_ = OPEN_GATE;
                }
                liftGate();
            }
            else if (actionFlag_ == OPEN_GATE)
            {
                LCD.print(0x14, "OPEN GATE", 9);
                tape.set(COLOR_BLUE);
                if (delayTimer.wait(700))
                {
                    actionFlag_ = EXTRUDE;
                }
                openGate();
            }
            else if (actionFlag_ == EXTRUDE)
            {
                LCD.print(0x14, "DISCHARGE", 9);
                tape.set(COLOR_MAGENTA);
                catchTowel();
                pushWall();
                if (extrudeTowel())
                {
                    if (delayTimer.wait(300))
                    {
                        processFlag_ = END;
                    }
                }
            }
        }
        else
        {
            tape.blink(COLOR_YELLOW, 200);
            action.move({0, -400});
        }
    }
    else if (processFlag_ == END)
    {
        LCD.print(0x14, "END", 9);
        tape.set(COLOR_BLUE);
        if (odom.getNow().y > 300+200 or notCollecting_)
        {
            chassis.brake();
            if (towel.resetElevator() and storeTowel())
            {
                if (delayTimer.wait(500))
                {
                    reset();
                    return true;
                }
                closeGate();
            }
        }
        else
        {
            tape.set(COLOR_BLUE);
            action.move({0, 200});
        }
        towel.resetElevator();
        pullWall();
        storeTowel();
    }

    return false;
}

void Collection::checkActionRegularly()
{
    if (isElevating_)
    {
        if (towel.elevateElevatorFor1500())
        {
            hasSetHeight_ = true;
            isElevating_ = false;
        }
    }
    if (isLowering_)
    {
        if (towel.LowerElevator(2500))
        {
            isLowering_ = false;
        }
    }
}

Collection::TOWEL_STATUS Collection::getTowelPosition()
{
    String data;
    thinkpadTimeoutTimer_.reset();
    makeARequestToSend();
    while (1)
    {
        if (serialThinkpad_.available() > 0)
        {
            char c = serialThinkpad_.read();
            if (c != '\n')
            {
                data += c;
                continue;
            }
            
            String pos = data.substring(1);
            if (isNumber(pos))
            {
                position_ = pos.toInt();
                if (data.indexOf('x') == 0)
                {
                    if (isSameColor_)
                    {
                        if (abs((zone_==ZONE_BLUE ? odom.getNow().y+position_ : odom.getNow().y-position_)-positionPrev_) <= 50)
                        {
                            if (zone_ == ZONE_BLUE)
                            {
                                if (side_ == SIDE_BACK and position_ > 0) 
                                {
                                    return TOWEL_STATUS::ERR_NOTOWEL;
                                }
                                else if (side_ == SIDE_FRONT and position_ < 0)
                                {
                                    return TOWEL_STATUS::ERR_NOTOWEL;
                                }
                            }
                            else
                            {
                                if (side_ == SIDE_BACK and position_ < 0)
                                {
                                    return TOWEL_STATUS::ERR_NOTOWEL;
                                }
                                else if (side_ == SIDE_FRONT and position_ > 0)
                                {
                                    return TOWEL_STATUS::ERR_NOTOWEL;
                                }
                            }
                        }
                    }
                    return TOWEL_STATUS::MOVE;
                }
                else if (data.indexOf('p') == 0)
                {
                    return TOWEL_STATUS::PULL;
                }
                else if (data.indexOf('c') == 0)
                {
                    return TOWEL_STATUS::CATCH;
                }
                else if (data.indexOf('r') == 0)
                {
                    return TOWEL_STATUS::REMOVE;
                }
                else // 'n'
                {
                    return TOWEL_STATUS::ERR_NOTOWEL;
                }
            }
            else
            {
                return TOWEL_STATUS::ERR_NODATA;
            }
        }
        if (thinkpadTimeoutTimer_.wait(50))
        {
            return TOWEL_STATUS::ERR_NODATA;
        }
    }
}

void Collection::stopTimeoutTimer()
{
    timeoutTimer_.stop();
    timeoutTimerIsStopped_ = true;
    hasCorrected_ = false;
}

void Collection::makeARequestToSend()
{
    digitalWrite(PIN_SERIAL_THINKPAD_RTS, LOW);
}

void Collection::withdrawARequestToSend()
{
    digitalWrite(PIN_SERIAL_THINKPAD_RTS, HIGH);
}

bool Collection::examineWhetherThisTowelIsMine()
{
    digitalWrite(PIN_UV_LED, HIGH);
    delay(100);
    int digitalVal[5] = {};
    for (int i = 0; i < 5; i++)
    {
        digitalVal[i] = analogRead(PIN_PHOTO_TRANSISTOR);
        delay(1);
    }
    std::sort(std::begin(digitalVal), std::end(digitalVal));
    digitalWrite(PIN_UV_LED, LOW);

    if (digitalVal[2] <= photoTransistorThreshold_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Collection::extendArm(int length)
{
    int devLength = length - extenderEncorder_.getCount()*RATIO_OF_LENGTH_COUNT;
    if ((devLength >= -15 and devLength <= 15) or not digitalRead(PIN_LIMIT_ARM_EXTEND))
    {
        extenderMotor_.setPWM(0);
        return true;
    }
    else
    {
        extenderMotor_.setPWM(constrain(fabs(devLength*KP_EXTEND), 40, 80)*setSign(-devLength));
        return false;
    }
}

bool Collection::shortenArm()
{
    if (not digitalRead(PIN_LIMIT_ARM_SHORTEN))
    {
        extenderMotor_.setPWM(0);
        extenderEncorder_.reset();
        return true;
    }
    else
    {
        extenderMotor_.setPWM(extenderEncorder_.getCount()*RATIO_OF_LENGTH_COUNT>= 150 ? 100 : 80);
        return false;
    }
}

void Collection::stopExtendAndShorten()
{
    extenderMotor_.setPWM(0);
}

void Collection::stretchCatcher()
{
    stretcherSolenoid_.open();
}

void Collection::withdrawCatcher()
{
    stretcherSolenoid_.close();
}

void Collection::catchTowel()
{
    catcherSolenoid_.open();
}

void Collection::releaseTowel()
{
    catcherSolenoid_.close();
}

void Collection::openGate()
{
    openerSolenoid_.open();
}

void Collection::closeGate()
{
    openerSolenoid_.close();
}

void Collection::liftGate()
{
    lifterSolenoid_.open();
}

void Collection::dropGate()
{
    lifterSolenoid_.close();
}

void Collection::pushWall()
{
    pusherSolenoid_.open();
}

void Collection::pullWall()
{
    pusherSolenoid_.close();
}

bool Collection::extrudeTowel()
{
    if (digitalRead(PIN_LIMIT_EXTRUDE))
    {
        extruderRelayMotor_.brake();
        return true;
    }
    else
    {
        extruderRelayMotor_.drive(CW);
        return false;
    }
}

bool Collection::storeTowel()
{
    if (digitalRead(PIN_LIMIT_STORE))
    {
        extruderRelayMotor_.brake();
        return true;
    }
    else
    {
        extruderRelayMotor_.drive(CCW);
        return false;
    }
}

void Collection::stopExtrude()
{
    extruderRelayMotor_.brake();
}

void Collection::moveBackMechanismToInitialPosition()
{
    int endFlag = 0;
    while (not shortenArm())
    {
        if (ui.escape())
        {
            stopExtendAndShorten();
            return;
        }
    }
    while (1)
    {
        if (endFlag == 0b11)
        {
            return;
        }
        if (extendArm(250) and towel.resetElevator())
        {
            endFlag |= 1 << 0;
        }
        if (storeTowel())
        {
            endFlag |= 1 << 1;
        }
        if (ui.escape())
        {
            towel.stopElevateAndLower();
            stopExtendAndShorten();
            stopExtrude();
            return;
        }
    }
}

void Collection::checkSensorStatus()
{
    LCD.print(0x00, extenderEncorder_.getCount(), 5);
    LCD.print(0x06, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT_SIDE));
    LCD.print(0x08, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT_SIDE));
    LCD.print(0x0a, analogRead(PIN_PHOTO_TRANSISTOR), 4);
    LCD.print(0x12, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_LEFT));
    LCD.print(0x14, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_FRONT_RIGHT));
    LCD.print(0x16, digitalRead(PIN_LIMIT_EXTRUDE));
    LCD.print(0x18, digitalRead(PIN_LIMIT_STORE));
    LCD.print(0x1a, digitalRead(PIN_LIMIT_ARM_EXTEND));
    LCD.print(0x1c, digitalRead(PIN_LIMIT_ARM_SHORTEN));
    SerialUSB.println(analogRead(PIN_PHOTO_TRANSISTOR));
}

void Collection::debug(bool(&status)[24])
{
    checkSensorStatus();

    if (status[MOTOR0_FORWARD])      extrudeTowel();
    else if (status[MOTOR0_REVERSE]) storeTowel();
    else                             stopExtrude();
    
    if (status[MOTOR1_FORWARD])      extendArm(99999);
    else if (status[MOTOR1_REVERSE]) shortenArm();
    else                             stopExtendAndShorten();

    if (status[TOGGLE0])             catchTowel();
    else                             releaseTowel();

    if (status[TOGGLE1])             stretchCatcher();
    else                             withdrawCatcher();

    if (status[TOGGLE2])             liftGate();
    else                             dropGate();

    if (status[TOGGLE3])             openGate();
    else                             closeGate();

    if (status[TOGGLE4])             pushWall();
    else                             pullWall();

    if (status[TOGGLE5])             digitalWrite(PIN_UV_LED, HIGH);
    else                             digitalWrite(PIN_UV_LED, LOW);


}

void Collection::stopDebug()
{
    stopExtendAndShorten();
    stopExtrude();
    releaseTowel();
    withdrawCatcher();
    closeGate();
    pullWall();
    digitalWrite(PIN_UV_LED, LOW);
}

void Collection::reset()
{
    withdrawARequestToSend();
    timeoutTimer_.stop();
    processFlag_ = STANDBY;
    actionFlag_ = SHORTEN;
    timeoutTimerIsStopped_ = true;
    timeoutFlag_ = false;
    hasCorrected_ = false;
    hasSetHeight_ = false;
    collectFlagTimes_ = 0;
    removeFlagTimes_ = 0;
    hasExtended_ = false;
    catched_ = false;
    hasExamined_ = false;
    notCollecting_ = true;
    isMyTowel_ = false;
    somethingWentWrong_ = false;
    seconds_ = 0;
    position_ = 0;
    positionPrev_ = 0;
    isElevating_ = false;
    isLowering_ = false;
    extenderMotor_.setPWM(0);
    extruderRelayMotor_.brake();
    extenderEncorder_.reset();
    catcherSolenoid_.free();
    withdrawCatcher();
    dropGate();
    closeGate();
    pullWall();
    clearSerialBuffer(serialThinkpad_);
    digitalWrite(PIN_UV_LED, LOW);
}

Collection collection;