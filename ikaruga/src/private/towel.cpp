#include "towel.hpp"
#include "../common/chassis.hpp"
#include "../common/definition.hpp"
#include "../common/i2c_lcd.hpp"
#include "../common/odometry.hpp"
#include "../common/tape_led.hpp"
#include "../common/sbdbt.hpp"
#include "action.hpp"
#include "user_interface.hpp"
#include "sheet.hpp"

Towel::Towel() :
    elevatorEncorder_(5, 4),
    elevatorMotor_(5),
    extenderMotor_(6),
    gripperSolenoid_(8, 11),
    lifterSolenoid_(10),
    swingSolenoid_(9),
    completeTowel_({
                     {false, false, false},
                     {false, false}
                   }),
    pointPoleBase_({
                     { // ZONE_BLUE
                       {{1800+200, 5400+300-480}, {3300+200, 5400+300-480}, {3300+200, 3350+300-480}},
                       {{1800+200, 5400+300-480}, {3300+200, 5400+300-480}}
                     },
                     { // ZONE_RED
                       {{3300+200, 5400+300-480}, {1850+200, 5400+300-480}, {1850+200, 3350+300-480}},
                       {{3300+200, 5400+300-480}, {1850+200, 5400+300-480}}
                     }
                   }),
    pointHangOut_({
                    { // ZONE_BLUE
                      {{2000+200, 5400+300-280}, {2955+200, 5400+300-280}, {2950+200, 3350+300-280}},
                      {{1940+200, 5400+300-280}, {3220+200, 5400+300-280}}
                    },
                    { // ZONE_RED
                      {{3100+200, 5400+300-280}, {2130+200, 5400+300-280}, {2135+200, 3350+300-280}},
                      {{3150+200, 5400+300-280}, {1890+200, 5400+300-280}}
                    }
                  }),
    routeSZ2ToTowel_({
                       { // ZONE_BLUE
                         {
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {2000+200, 5400+300}, {2000+200, 5100+300}},
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {3300+200, 5400+300}, {3300+200, 5100+300}},
                           {{0, 0}, {100+200, 0+300}, {100+200, 3500+300}, {3300+200, 3400+300}, {3300+200, 3100+300}}
                         },
                         {
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {1800+200, 5400+300}, {1800+200, 5100+300}},
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {3300+200, 5400+300}, {3300+200, 5100+300}}
                         }
                       },
                       { // ZONE_RED
                         {
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {3100+200, 5400+300}, {3100+200, 5100+300}},
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {1800+200, 5400+300}, {1800+200, 5100+300}},
                           {{0, 0}, {100+200, 0+300}, {100+200, 3500+300}, {1800+200, 3400+300}, {1800+200, 3100+300}}
                         },
                         {
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {3300+200, 5400+300}, {3300+200, 5100+300}},
                           {{0, 0}, {100+200, 0+300}, {100+200, 5500+300}, {1800+200, 5400+300}, {1800+200, 5100+300}}
                         }
                       }
                     }),
    routeTowelToSZ2_({
                       { // ZONE_BLUE
                         {
                           {{1800+200, 5500+300}, {0+200, 5500+300}, {0+200, 0+300}},
                           {{2950+200, 5500+300}, {0+200, 5500+300}, {0+200, 0+300}},
                           {{2950+200, 3500+300}, {0+200, 3500+300}, {0+200, 0+300}}
                         },
                         {
                           {{1910+200, 5500+300}, {0+200, 5500+300}, {0+200, 0+300}},
                           {{3210+200, 5500+300}, {0+200, 5500+300}, {0+200, 0+300}}
                         }
                       },
                       { // ZONE_RED
                         {
                           {{3200+200, 5500+300}, {0+200, 5500+300}, {-100+200, 0+300}},
                           {{2150+200, 5500+300}, {0+200, 5500+300}, {-100+200, 0+300}},
                           {{2150+200, 3500+300}, {0+200, 3500+300}, {-100+200, 0+300}}
                         },
                         {
                           {{3210+200, 5500+300}, {0+200, 5500+300}, {-100+200, 0+300}},
                           {{1910+200, 5500+300}, {0+200, 5500+300}, {-100+200, 0+300}}
                         }
                       }
                     })
{}

void Towel::initialize()
{
    pinMode(PIN_LIMIT_ELEVATE_BOTTOM, INPUT);
    pinMode(PIN_LIMIT_ELEVATE_TOP, INPUT);
    pinMode(PIN_LIMIT_EXPAND, INPUT);
    pinMode(PIN_LIMIT_EXPAND_BY_DEFAULT, INPUT);
    pinMode(PIN_LIMIT_SHRINK, INPUT);
    pinMode(PIN_LIMIT_POLE_GUIDE_LEFT, INPUT);
    pinMode(PIN_LIMIT_POLE_GUIDE_RIGHT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_LEFT, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_RIGHT, INPUT);
    pinMode(PIN_LIMIT_HAND_GUIDE, INPUT);
    reset();
}

bool Towel::getStarted()
{
    zone_ = ui[ZONE];
    round_ = ui[ROUND];
    workSheet_ = ui[MODE_SHEET];

    bool hasDecided = false;
    for (int i = 0; i < completeTowel_[round_].size(); i++)
    {
        completeTowel_[round_][i] = not ui[TOWEL0+i];
        if (not hasDecided and ui[TOWEL0+i])
        {
            towelIndex_ = i;
            hasDecided = true;
        }
    }

    if (not hasDecided)
    {
        LCD.print(0x14, "NO TOWEL", 9);
        tape.set(COLOR_RED);
        return false;
    }

    gripTowel();
    swingUp();
    isFirstTowel_ = true;
    hasDecided_ = true;
    hasReloaded_ = true;
    if (round_ == ROUND_PRELIMINARY and towelIndex_ == 0) hasCorrectedYAxis_ = true;
    actionFlag_ = SWING_DOWN;
    action.setYawTarget(180);
    if (towelIndex_ == 2 and not digitalRead(PIN_LIMIT_ELEVATE_BOTTOM)) isLowering_ = true;
    if (not ui.isTestingOutWork() and round_ == ROUND_PRELIMINARY and towelIndex_ == 0)
    {
        isAtPoint_ = true;
        hasCorrected_ = true;
    }
    if (not ui.isTestingOutWork())
    {
        processFlag_ = START;
    }
    else
    {
        if (towelIndex_ == 2) isElevatingFor1000_ = true;
        else                  isElevatingFor1500_ = true;
        isAtPoint_ = true;
        processFlag_ = RELOAD_AND_APPROACH_POLE;
    }
    return true;
}

bool Towel::work(MyTimer& timer)
{
    if (processFlag_ == STANDBY)
    {
        return false;
    }

    checkActionRegularly();

    if (processFlag_ == START)
    {
        LCD.print(0x14, "START", 9);
    
        if (action.follow(routeSZ2ToTowel_[zone_][round_][towelIndex_], Action::MUTUAL))
        {
            timer.reset();
            processFlag_ = RELOAD_AND_APPROACH_POLE;
        }
        if (not action.isPaused())
        {
            if (not isLowering_ and timer.wait(1000))
            {
                if (towelIndex_ == 2) isElevatingFor1000_ = true;
                else                  isElevatingFor1500_ = true;
            }
        }
    }
    else if (processFlag_ == RELOAD_AND_APPROACH_POLE)
    {
        //LCD.print(0x14, "RLD&APPRCH", 9);
        if (not hasReloaded_ and not hasApproached_)
        {
            tape.blinkMultiColor(COLOR_BLUE, COLOR_YELLOW, 150);
        }
        else if (not hasReloaded_)
        {
            tape.set(COLOR_BLUE);
        }
        else if (not hasApproached_)
        {
            tape.set(COLOR_YELLOW);
        }

        if (not hasDecided_)
        {
            bool noTowel = true;
            for (int i = 0; i < completeTowel_[round_].size(); i++)
            {
                if (not completeTowel_[round_][i])
                {
                    towelIndex_ = i;
                    noTowel = false;
                    break;
                }
            }
            if (noTowel)
            {
                isLowering_ = true;
                processFlag_ = workSheet_ ? START_SHEET : (not ui.isTestingOutWork() ? RETURN : END);
                return false;
            }
            hasDecided_ = true;
            isShrinkingByDefault_ = false;
            isShrinking_ = true;
            if (towelIndex_ == 2) 
            {
                isMovingForTowel2_ = true;
                hasSetheight_ = false;
                isLowering_ = true;
            }
            else
            {
                isElevatingFor1500_ = true;
            }
        }

        if (not hasSetheight_)
        {
            if (not isLowering_)
            {
                if (towelIndex_ == 2) 
                {
                    if (not isMovingForTowel2_ or hasPassedViaPoint_)
                    {
                        isElevatingFor1000_ = true;
                    }
                }
                else
                {
                    isElevatingFor1500_ = true;
                }
            }
            else
            {
                return false;
            }
        }

        if (isFirstTowel_ and not hasCorrectedYAxis_)
        {
            if (not digitalRead(PIN_LIMIT_POLE_GUIDE_LEFT) and not digitalRead(PIN_LIMIT_POLE_GUIDE_RIGHT))
            {
                chassis.brake();
                if (round_ == ROUND_PRELIMINARY and towelIndex_ == 2)
                {
                    odom.correct({odom.getNow().x, 2700+100+300});
                }
                else
                {
                    odom.correct({odom.getNow().x, 4700+100+300});
                }
                hasCorrectedYAxis_ = true;
            }
            else
            {
                action.move({0, -200});
            }
            return false;
        }

        if (not hasReloaded_)
        {
            if (reloadTowel(timer))
            {
                hasReloaded_ = true;
            }
        }
        if (not hasApproached_)
        {
            if (not isFirstTowel_ and not (round_ == ROUND_PRELIMINARY and towelIndex_ == 1))
            {
                isAtPoint_ = true;
                hasCorrected_ = true;
            }

            if (not isAtPoint_)
            {
                if (hasReloaded_)
                {
                    if (action.moveToward(pointPoleBase_[zone_][round_][towelIndex_]))
                    {
                        hasCorrected_ = false;
                        isAtPoint_ = true;
                    }
                }
                else
                {
                    action.moveToward(pointPoleBase_[zone_][round_][towelIndex_]+DIST_MOVE_BACKWARD);
                }
            }
            else
            {
                if (not hasCorrected_)
                {
                    if (towelIndex_ == 0)
                    {
                        if (correctOdometryWithLeftLimit(zone_==ZONE_BLUE ? -1150-420-200 : 3950-420+200, timer))
                        {
                            hasCorrected_ = true;
                        }
                    }
                    else if (towelIndex_ == 1 or towelIndex_ == 2)
                    {
                        if (correctOdometryWithRightLimit(zone_==ZONE_BLUE ? -3950+420-200 : 1150+420+200, timer))
                        {
                            hasCorrected_ = true;
                        }
                    }
                }
                else
                {
                    if (hasReloaded_ and (not isMovingForTowel2_ or hasPassedViaPoint_))
                    {
                        if (action.moveTo(pointHangOut_[zone_][round_][towelIndex_]))
                        {
                            hasApproached_ = true;
                        }
                        isExpanding_ = true;
                    }
                    else
                    {
                        if (isMovingForTowel2_)
                        {
                            if (not isLowering_ and action.moveToward(pointHangOut_[zone_][round_][towelIndex_]+DIST_MOVE_BACKWARD, 1000))
                            {
                                isMovingForTowel2_ = false;
                            }
                            if (odom.getNow().y <= 3700+300) hasPassedViaPoint_ = true;
                        }
                        else
                        {
                            action.moveToward(pointHangOut_[zone_][round_][towelIndex_]+DIST_MOVE_BACKWARD);
                        }
                    }
                }
            }
        }

        if (hasReloaded_ and hasApproached_)
        {
            hasDecided_ = false;
            isAtPoint_ = false;
            hasReloaded_ = false;
            hasCorrected_ = false;
            hasApproached_ = false;
            processFlag_ = HANG_OUT_TOWEL;
        }
    }
    else if (processFlag_ == HANG_OUT_TOWEL)
    {
        if (not hasCorrected_)
        {
            if (hasSetheight_ and not isExpanding_)
            {
                if (actionFlag_ == SWING_DOWN)
                {
                    swingDown();
                    actionFlag_ = RELEASE_TOWEL;
                }

                if (not digitalRead(PIN_LIMIT_POLE_GUIDE_LEFT) and not digitalRead(PIN_LIMIT_POLE_GUIDE_RIGHT))
                {
                    chassis.brake();
                    if (round_ == ROUND_PRELIMINARY and towelIndex_ == 0) towel0_ = true;
                    hasCorrected_ = true;
                }
                else
                {
                    tape.blink(COLOR_BLUE, 100);
                    action.move({0, -200});
                }
            }
            else
            {
                LCD.print(0x14, "EXPANDING", 9);
                tape.set(COLOR_BLUE);
                chassis.brake();
            }
        }
        else
        {
            tape.set(COLOR_MAGENTA);
            if (towel0_)
            {
                if (not digitalRead(PIN_LIMIT_HAND_GUIDE))
                {
                    chassis.brake();
                    towel0_ = false;
                }
                else
                {
                    action.move({zone_==ZONE_BLUE ? -200 : 200, 0});
                }
                isExpandingByDefault_ = true;
            }
            else
            {
                if (hangOutTowel(timer))
                {
                    odom.correct({odom.getNow().x, (round_ == ROUND_PRELIMINARY and towelIndex_ == 2) ? 2700+100+300 : 4700+100+300});
                    hasCorrected_ = false;
                    completeTowel_[round_][towelIndex_] = true;
                    processFlag_ = LEAVE_POLE;
                }
            }  
        }
    }
    else if (processFlag_ == LEAVE_POLE)
    {
        LCD.print(0x14, "LEAVE", 9);
        if (not hasLowered_)
        {
            tape.set(COLOR_MAGENTA);
            if (odom.getNow().y >= ((towelIndex_ == 2) ? 2700+100+300+100 : 4700+100+300+100))
            {
                chassis.brake();
                isShrinkingByDefault_ = true;
                if (LowerElevator(1500))
                {
                    hasLowered_ = true;
                }
            }
            else
            {
                action.move({0, 200});
            }
        }
        else
        {
            tape.set(COLOR_YELLOW);
            if (action.moveToward(pointHangOut_[zone_][round_][towelIndex_]+DIST_MOVE_BACKWARD_SLIGHTLY))
            {
                hasLowered_ = false;
                isFirstTowel_ = false;
                processFlag_ = RELOAD_AND_APPROACH_POLE;
            }
        }
    }
    else if (processFlag_ == START_SHEET)
    {
        LCD.print(0x14, "SHRNK&LWR", 9);
        tape.set(COLOR_BLUE);
        if (not isLowering_ and not isShrinkingByDefault_)
        {
            LCD.clearAll();
            sheet.startFromTheMiddle(towelIndex_);
            reset();
            return false;
        }
    }
    else if (processFlag_ == RETURN)
    {
        LCD.print(0x14, "RETURN", 9);
        if (action.follow(routeTowelToSZ2_[zone_][round_][towelIndex_], Action::MUTUAL))
        {
            processFlag_ = END;
        }
    }
    else if (processFlag_ == END)
    {
        reset();
        return true;
    }
    return false;
}

bool Towel::reloadTowel(MyTimer& timer)
{
    if (actionFlag_ == SWING_DOWN)
    {
        if (not isShrinking_)
        {
            LCD.print(0x14, "SWINGDOWN", 9);
            if (timer.wait(200))
            {
                actionFlag_ = LIFT_MAGAZINE;
            }
            swingDown();
        }
        else
        {
            LCD.print(0x14, "SHRINKING", 9);
        }
    }
    else if (actionFlag_ == LIFT_MAGAZINE)
    {
        LCD.print(0x14, "LIFT", 9);
        if (timer.wait(500))
        {
            actionFlag_ = GRIP_TOWEL;
        }
        liftMagazine();
    }
    else if (actionFlag_ == GRIP_TOWEL)
    {
        LCD.print(0x14, "GRIP", 9);
        if (timer.wait(500))
        {
            actionFlag_ = DROP_MAGAZINE;
        }
        gripTowel();
    }
    else if (actionFlag_ == DROP_MAGAZINE)
    {
        LCD.print(0x14, "DROP", 9);
        if (timer.wait(500))
        {
            actionFlag_ = SWING_UP;
        }
        dropMagazine();
    }
    else if (actionFlag_ == SWING_UP)
    {
        LCD.print(0x14, "SWINGUP", 9);
        if (timer.wait(500))
        {
            actionFlag_ = SWING_DOWN;
            isExpandingByDefault_ = true;
            return true;
        }
        swingUp();
    }
    return false;
}

bool Towel::hangOutTowel(MyTimer& timer)
{
    if (actionFlag_ == SWING_DOWN)
    {
        LCD.print(0x14, "SWINGDOWN", 9);
        if (timer.wait(500))
        {
            actionFlag_ = RELEASE_TOWEL;
        }
        swingDown();
    }
    else if (actionFlag_ == RELEASE_TOWEL)
    {
        LCD.print(0x14, "RELEASE_TOWEL", 9);
        if (timer.wait(500))
        {
            actionFlag_ = SWING_UP;
        }
        releaseTowel();
    }
    else if (actionFlag_ == SWING_UP)
    {
        LCD.print(0x14, "SWINGUP", 9);
        if (timer.wait(500))
        {
            actionFlag_ = SWING_DOWN;
            return true;
        }
        swingUp();
    }

    return false;
}

void Towel::checkActionRegularly()
{
    if (isElevatingFor1000_)
    {
        if (elevateElevator(COUNT_1000))
        {
            hasSetheight_ = true;
            isElevatingFor1000_ = false;
        }
    }
    if (isElevatingFor1500_)
    {
        if (elevateElevator(COUNT_1500))
        {
            hasSetheight_ = true;
            isElevatingFor1500_ = false;
        }
    }
    if (isLowering_)
    {
        if (resetElevator())
        {
            isLowering_ = false;
        }
    }
    if (isExpanding_)
    {
        if (extendArms())
        {
            isExpanding_ = false;
        }
    }
    if (isExpandingByDefault_)
    {
        if (extendArmsByDefault())
        {
            isExpandingByDefault_ = false;
        }
    }
    if (isShrinking_)
    {
        if (shrinkArms())
        {
            isShrinking_ = false;
        }
    }
    if (isShrinkingByDefault_)
    {
        if (shrinkArmsByDefault())
        {
            isShrinkingByDefault_ = false;
        }
    }
}

bool Towel::elevateElevator(long count)
{
    long devCount = count - elevatorEncorder_.getCount();
    if ((devCount >= -40 and devCount <= 40) or digitalRead(PIN_LIMIT_ELEVATE_TOP))
    {
        elevatorMotor_.setPWM(0);
        countTop_ = elevatorEncorder_.getCount();
        return true;
    }
    else
    {
        elevatorMotor_.setPWM(constrain(abs(devCount*KP_ELEVATE), 30, 100)*setSign(-devCount));
        return false;
    }
}

bool Towel::elevateElevatorFor1500()
{
    if (elevateElevator(COUNT_1500))
    {
        return true;
    }
    return false;
}

bool Towel::LowerElevator(long count)
{
    if (elevatorEncorder_.getCount() <= countTop_-count or digitalRead(PIN_LIMIT_ELEVATE_BOTTOM))
    {
        elevatorMotor_.setPWM(0);
        return true;
    }
    else
    {
        elevatorMotor_.setPWM(100);
        return false;
    }   
}

bool Towel::resetElevator()
{
    if (digitalRead(PIN_LIMIT_ELEVATE_BOTTOM))
    {
        elevatorMotor_.setPWM(0);
        elevatorEncorder_.reset();
        return true;
    }
    else
    {
        elevatorMotor_.setPWM(100);
        return false;
    }
}

void Towel::stopElevateAndLower()
{
    elevatorMotor_.setPWM(0);
}

bool Towel::correctOdometryWithLeftLimit(double correctX, MyTimer& correctionTimer)
{
    if (not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_LEFT))
    {
        action.move({zone_==ZONE_BLUE ? -100 : 100, 0});
        if (correctionTimer.wait(500))
        {
            chassis.brake();
            odom.correct({correctX, odom.getNow().y});
            return true;
        }
    }
    else
    {
        correctionTimer.reset();
        action.move({zone_==ZONE_BLUE ? -250 : 250, 0});
        return false;
    }
}

bool Towel::correctOdometryWithRightLimit(double correctX, MyTimer& correctionTimer)
{
    if (not digitalRead(PIN_LIMIT_CHASSIS_GUIDE_RIGHT))
    {
        action.move({zone_==ZONE_BLUE ? 100 : -100, 0});
        if (correctionTimer.wait(500))
        {
            chassis.brake();
            odom.correct({correctX, odom.getNow().y});
            return true;
        }
    }
    else
    {
        correctionTimer.reset();
        action.move({zone_==ZONE_BLUE ? 250 : -250, 0});
        return false;
    }
}

bool Towel::extendArms()
{
    if (digitalRead(PIN_LIMIT_EXPAND))
    {
        extenderMotor_.setPWM(0);
        return true;
    }
    else
    {
        extenderMotor_.setPWM(-35);
        return false;
    }
}

bool Towel::extendArmsByDefault()
{
    if (not digitalRead(PIN_LIMIT_EXPAND_BY_DEFAULT) or digitalRead(PIN_LIMIT_EXPAND))
    {
        extenderMotor_.setPWM(0);
        return true;
    }
    else
    {
        extenderMotor_.setPWM(-20);
        return false;
    }
}

bool Towel::shrinkArms()
{
    if (digitalRead(PIN_LIMIT_SHRINK))
    {
        extenderMotor_.setPWM(0);
        return true;
    }
    else
    {
        extenderMotor_.setPWM(35);
        return false;
    }
}

bool Towel::shrinkArmsByDefault()
{
    if (not digitalRead(PIN_LIMIT_EXPAND_BY_DEFAULT) or digitalRead(PIN_LIMIT_SHRINK))
    {
        extenderMotor_.setPWM(0);
        return true;
    }
    else
    {
        extenderMotor_.setPWM(20);
        return false;
    }
}

void Towel::stopExpandAndShrink()
{
    extenderMotor_.setPWM(0);
}

void Towel::gripTowel()
{
    gripperSolenoid_.open();
}

void Towel::releaseTowel()
{
    gripperSolenoid_.close();
}

void Towel::swingUp()
{
    swingSolenoid_.open();
    swingStatus_ = 1;
}

void Towel::swingDown()
{
    swingSolenoid_.close();
    swingStatus_ = 0;
}

void Towel::liftMagazine()
{
    lifterSolenoid_.open();
}

void Towel::dropMagazine()
{
    lifterSolenoid_.close();
}

void Towel::moveBackMechanismToInitialPosition()
{
    int endFlag = 0;
    while (1)
    {
        if (endFlag == 0b111) 
        {
            return;
        }
        if (endFlag&(1<<0))
        {
            if (extendArmsByDefault())
            {
                endFlag |= 1 << 1;
            } 
        }
        else if (shrinkArms()) 
        {
            endFlag |= 1 << 0;
        }
        if (resetElevator()) 
        {
            endFlag |= 1 << 2;
        }
        if (ui.escape())
        {
            stopExpandAndShrink();
            stopElevateAndLower();
            return;
        }
    }
}

void Towel::checkSensorStatus()
{
    LCD.print(0x00, elevatorEncorder_.getCount(), 5);
    LCD.print(0x06, digitalRead(PIN_LIMIT_ELEVATE_BOTTOM));
    LCD.print(0x08, digitalRead(PIN_LIMIT_ELEVATE_TOP));
    LCD.print(0x0a, digitalRead(PIN_LIMIT_EXPAND));
    LCD.print(0x10, digitalRead(PIN_LIMIT_EXPAND_BY_DEFAULT));
    LCD.print(0x12, digitalRead(PIN_LIMIT_SHRINK));
    LCD.print(0x14, digitalRead(PIN_LIMIT_POLE_GUIDE_LEFT));
    LCD.print(0x16, digitalRead(PIN_LIMIT_POLE_GUIDE_RIGHT));
    LCD.print(0x18, digitalRead(PIN_LIMIT_HAND_GUIDE));
    LCD.print(0x1a, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_LEFT));
    LCD.print(0x1c, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_RIGHT));
}

void Towel::debug(bool(&status)[24])
{
    checkSensorStatus();

    if (status[MOTOR2_FORWARD])      elevateElevator(COUNT_1500);
    else if (status[MOTOR3_FORWARD]) elevateElevator(COUNT_1000);
    else if (status[MOTOR2_REVERSE]) resetElevator();
    else                             stopElevateAndLower();

    if (status[MOTOR0_FORWARD])      extendArms();
    else if (status[MOTOR0_REVERSE]) shrinkArms();
    else if (status[MOTOR1_FORWARD]) extendArmsByDefault();
    else if (status[MOTOR1_REVERSE]) shrinkArmsByDefault();
    else                             stopExpandAndShrink();

    if (status[TOGGLE0])             gripTowel();
    else                             releaseTowel();

    if (status[TOGGLE1])             swingDown();
    else                             swingUp();

    if (status[TOGGLE2])             liftMagazine();
    else                             dropMagazine();
}

void Towel::stopDebug()
{
    stopElevateAndLower();
    stopExpandAndShrink();
    releaseTowel();
    swingUp();
    dropMagazine();
}

void Towel::reset()
{
    processFlag_ = STANDBY;
    actionFlag_ = LIFT_MAGAZINE;
    towelIndex_ = 0;
    isFirstTowel_ = true;
    hasSetheight_ = false;
    hasDecided_ = false;
    isAtPoint_ = false;
    hasPassedViaPoint_ = false;
    isMovingForTowel2_ = false;
    hasApproached_ = false;
    hasCorrectedYAxis_ = false;
    hasCorrected_ = false;
    hasReloaded_ = false;
    hasLowered_ = false;
    towel0_ = false;
    swingStatus_ = false;
    isElevatingFor1000_ = false;
    isElevatingFor1500_ = false;
    isLowering_ = false;
    isExpanding_ = false;
    isExpandingByDefault_ = false;
    isShrinking_ = false;
    isShrinkingByDefault_ = false;

    elevatorEncorder_.reset();
    elevatorMotor_.setPWM(0);
    extenderMotor_.setPWM(0);
    gripperSolenoid_.free();
    dropMagazine();
    if (initialPos_) swingDown();
    else             swingUp();
}

Towel towel;