#include "sheet.hpp"
#include <Arduino.h>
#include "../action.hpp"
#include "../base.hpp"
#include "../chassis.hpp"
#include "../controller.hpp"
#include "../definition.hpp"
#include "../i2c_lcd.hpp"
#include "../odometry.hpp"
#include "../sbdbt.hpp"
#include "../tape_led.hpp"
#include "tshirt.hpp"

Sheet::Sheet() :
setPoint_({
    {
        {550+100,8800}, //+300
        {250+100,8800},
        {250+100,9150},
        {2850+100,8800}, //+2600
        {2850+100,9150}
    },
    {
        {-2650+150,8800}, //+300
        {-2950+150,8800},
        {-2950+150,9150},
        {-350+150,8800}, //+2600
        {-350+150,9150}
    }
}),
correctPoint_({
    {
        {-1200,9300},
        {setPoint_[0][1].x,8700}
    },
    {
        {-3700,9300},
        {setPoint_[1][1].x,8700}
    }
}),
SZToHalfwayRoute_({
    { {3600,0}, {400,3000} },
    { {-3600,0}, {-400,3000} }
}),
HZToCornerRoute_({
    { {0,0}, {400,3000}, {-900,3000}, {-1000,9100} },
    { {0,0}, {-400,3000}, {900,3000}, {900,9000} }
}),
CornerToCornerRoute_(
    { {900,9000} , {-3500,9100} }
),
SheetToCornerRoute_({
    { setPoint_[0][4] , setPoint_[0][2] },
    { setPoint_[1][2] , setPoint_[1][4] }
}),
CornerToHZRoute_({
    { setPoint_[0][2], {-900,9150}, {-900,3200}, {400,3200}, {200,200} },
    { setPoint_[1][4], {900,9150}, {900,3200}, {-400,3200}, {-200,200} }
}),
timer_(base.timer_),
rogerEncoder_(5,4),
rogerMotor_(4),
sliderMotor_(5),
deliverySolenoid_(2),
gripperU1Solenoid_(8,9),
gripperU2Solenoid_(10,11),
gripper1Solenoid_(12,13),
gripper2Solenoid_(14,15),
hookSolenoid_(16,17),
yamatoSolenoid_(18)
{}

bool Sheet::work()
{
    if(flag_ == FLAG::GO_SZ)
    {
        lcd.print(0x10, "GO", 8);
        if(action.follow(SZToHalfwayRoute_[ZONE], PAUSE_NO_RESUME))
        {
            flag_ = FLAG::GO;
        }
    }
    else if(flag_ == FLAG::GO)
    {
        lcd.print(0x10, "GO", 8);
        if(action.follow(HZToCornerRoute_[ZONE]))
        {
            flag_ = ZONE_BLUE ? FLAG::GO_X : FLAG::CORRECT;
        }
    }
    else if(flag_ == FLAG::GO_X)
    {
        lcd.print(0x10, "GO", 8);
        if(action.follow(CornerToCornerRoute_, PAUSE_NONE))
        {
            flag_ = FLAG::CORRECT;
        }
    }
    else if(flag_ == FLAG::CORRECT)
    {
        lcd.print(0x10, "CORRECT", 8);
        tape.blink(COLOR_YELLOW);
        if(DUET) bt.transmit(COM_CODE::DO_COME);
        if(setPositionToCorrect())
        {
            if(DUET)
            {
                releaseSheet2();
                releaseUpperSheet2();
                flag_ = FLAG::WAIT_1;
            }
            else
            {
                flag_ = FLAG::SET_2;
            }
        }
    }
    else if(flag_ == FLAG::WAIT_1)
    {
        lcd.print(0x10, "WAIT", 8);
        tape.set(COLOR_MAGENTA);
        bt.transmit(COM_CODE::DO_COME);
        if(extendSlider())
        {
            if(!digitalRead(PIN_LIMIT_CHASSIS_DELIVER))
            {
                if(timer_.wait(500))
                {
                    flag_ = FLAG::DELIVER;
                }
            }
            else
            {
                timer_.reset();
            }
        }
    }
    else if(flag_ == FLAG::DELIVER)
    {
        lcd.print(0x10, "DELIVER", 8);
        tape.blink(COLOR_MAGENTA);
        if(shrinkSlider())
        {
            if(digitalRead(PIN_LIMIT_DELIVERY_TOUCH))
            {
                bt.flush();
                bt.transmit(COM_CODE::DO_GRIP);
                flag_ = FLAG::WAIT_2;
            }
            else
            {
                flag_ = FLAG::WAIT_1;
            }
        }
    }
    else if(flag_ == FLAG::WAIT_2)
    {
        lcd.print(0x10, "WAIT", 8);
        tape.set(COLOR_MAGENTA);
        if(bt.receive() == COM_CODE::GRIP_OK)
        {
            releaseDeliverySheet();
            bt.flush();
            flag_ = FLAG::WAIT_3;
        }
    }
    else if(flag_ == FLAG::WAIT_3)
    {
        lcd.print(0x10, "WAIT", 8);
        tape.set(COLOR_MAGENTA);
        if(bt.receive() == COM_CODE::MOVE_OK)
        {
            flag_ = FLAG::SET_1;
        }
    }
    else if(flag_ == FLAG::SET_1)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        adjustRoger(COUNT_ROGER_HOOK);
        shrinkSlider();
        if(action.moveTo(setPoint_[ZONE][0], 400))
        {
            if(adjustRoger(COUNT_ROGER_HOOK))
            {
                bt.flush();
                flag_ = FLAG::WAIT_4;
            }
        }
    }
    else if(flag_ == FLAG::WAIT_4)
    {
        lcd.print(0x10, "WAIT", 8);
        tape.set(COLOR_MAGENTA);
        if(bt.receive() == COM_CODE::SET_OK)
        {
            pinchDeliverySheet();
            flag_ = FLAG::SET_2;
        }
    }
    else if(flag_ == FLAG::SET_2)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        adjustRoger(COUNT_ROGER_HOOK);
        if(setPositionToHook())
        {
            if(adjustRoger(COUNT_ROGER_HOOK))
            {
                if(timer_.wait(500))
                {
                    flag_ = FLAG::HOOK;
                }   
            }
        }
    }
    else if(flag_ == FLAG::HOOK)
    {
        lcd.print(0x10, "HOOK", 8);
        tape.blink(COLOR_BLUE);
        extendYamato();
        extendHook();
        if(timer_.wait(500))
        {
            shrinkHook();
            flag_ = FLAG::RELEASE_1;
        }
    }
    else if(flag_ == FLAG::RELEASE_1)
    {
        lcd.print(0x10, "RELEASE", 8);
        tape.blink(COLOR_BLUE);
        releaseSheet1();
        if(timer_.wait(500))
        {
            if(DUET)
            {
                bt.transmit(COM_CODE::DO_LOWER);
                flag_ = FLAG::LOWER_1;
            }
            else
            {
                flag_ = FLAG::RELEASE_U;
            }
        }
    }
    else if(flag_ == FLAG::LOWER_1)
    {
        lcd.print(0x10, "LOWER", 8);
        tape.blink(COLOR_BLUE);
        if(adjustRoger(COUNT_ROGER_RELEASE_DUET))
        {
            if(timer_.wait(500))
            {
                bt.transmit(COM_CODE::DO_RELEASE);
                flag_ = FLAG::RELEASE_U;
            }
        }
    }
    else if(flag_ == FLAG::RELEASE_U)
    {
        lcd.print(0x10, "RELEASE", 8);
        tape.blink(COLOR_BLUE);
        releaseUpperSheet();
        if(timer_.wait(500))
        {
            flag_ = DUET ? FLAG::BACK_1 : FLAG::RAISE;
        } 
    }
    else if(flag_ == FLAG::BACK_1)
    {
        lcd.print(0x10, "BACK", 8);
        tape.blink(COLOR_YELLOW);
        if(action.moveTo(setPoint_[ZONE][2], 200))
        {
            bt.flush();
            flag_ = ZONE_RED ? FLAG::LOWER_3 : FLAG::WAIT_5;
        }
    }
    else if(flag_ == FLAG::WAIT_5)
    {
        lcd.print(0x10, "WAIT", 8);
        tape.set(COLOR_MAGENTA);
        lowerRoger();
        if(bt.receive() == COM_CODE::RETURN_OK)
        {
            flag_ = FLAG::RETURN_X;
        }
    }
    else if(flag_ == FLAG::RAISE)
    {
        lcd.print(0x10, "RAISE", 8);
        tape.blink(COLOR_BLUE);
        if(adjustRoger(COUNT_ROGER_UNFOLD))
        {
            flag_ = FLAG::SET_3;
        }
    }
    else if(flag_ == FLAG::SET_3)
    {
        lcd.print(0x10, "SET", 8);
        tape.blink(COLOR_YELLOW);
        if(setPositionToRelease())
        {
            shrinkHook();
            if(timer_.wait(500))
            {
                flag_ = FLAG::LOWER_2;
            }
        }
    }
    else if(flag_ == FLAG::LOWER_2)
    {
        lcd.print(0x10, "LOWER", 8);
        tape.set(COLOR_BLUE);
        if(adjustRoger(COUNT_ROGER_RELEASE_SOLO))
        {
            if(timer_.wait(500))
            {
                flag_ = FLAG::RELEASE_2;
            }
        }
    }
    else if(flag_ == FLAG::RELEASE_2)
    {
        lcd.print(0x10, "RELEASE", 8);
        tape.blink(COLOR_BLUE);
        releaseSheet2();
        if(timer_.wait(500))
        {
            flag_ = FLAG::BACK_2;
        }
    }
    else if(flag_ == FLAG::BACK_2)
    {
        lcd.print(0x10, "BACK", 8);
        tape.blink(COLOR_YELLOW);
        if(action.moveTo(setPoint_[ZONE][4], 200))
        {
            flag_ = FLAG::LOWER_3;
        }
    }
    else if(flag_ == FLAG::LOWER_3)
    {
        lcd.print(0x10, "LOWER", 8);
        tape.set(COLOR_BLUE);
        lowerRoger();
        if(timer_.wait(1000))
        {
            flag_ = ((ZONE_RED&&SOLO)||(ZONE_BLUE&&DUET)) ? FLAG::RETURN_X : FLAG::RETURN;
        }
    }
    else if(flag_ == FLAG::RETURN_X)
    {
        lcd.print(0x10, "RETURN", 8);
        lowerRoger();
        if(action.follow(SheetToCornerRoute_[ZONE], PAUSE_NONE))
        {
            flag_ = FLAG::RETURN;
        }
    }
    else if(flag_ == FLAG::RETURN)
    {
        lcd.print(0x10, "RETURN", 8);
        lowerRoger();
        if(action.follow(CornerToHZRoute_[ZONE]))
        {
            return true;
        }
    }

    return false;
}

bool Sheet::test()
{
    if(flag_ == FLAG::LOWER_3)
    {
        if(lowerRoger()) return true;
    }
    else
    {
        work();
    }

    return false;
}

bool Sheet::setPositionToCorrect()
{
    Vector2 vTrans =
    {
        !digitalRead(PIN_LIMIT_CHASSIS_TOUCH_X) ? 0 : -200,
        !digitalRead(PIN_LIMIT_CHASSIS_TOUCH_Y) ? 0 : 200,
    };

    if(!digitalRead(PIN_LIMIT_CHASSIS_TOUCH_X) && !digitalRead(PIN_LIMIT_CHASSIS_TOUCH_Y))
    {
        chassis.brake();
        odom.correct(correctPoint_[ZONE][0]);
        return true;
    }
    else
    {
        action.move(vTrans);
        return false;
    }
}

bool Sheet::setPositionToHook()
{
    Vector2 devPos = setPoint_[ZONE][1] - odom.getNow();

    Vector2 vTrans =
    {
        constrain(fabs(devPos.x)*1, 100, 400)*setSign(devPos.x, 10),
        100*setSign(devPos.y, 10),
    };

    if(fabs(devPos.x)<=50)
    {
        if(!digitalRead(PIN_LIMIT_CHASSIS_GUIDE_1))
        {
            if(fabs(devPos.x)<=10)
            {
                if(action.isYawDeviationAllowable())
                {
                    chassis.brake();
                    odom.correctY(correctPoint_[ZONE][1].y);
                    return true;
                }
                else
                {
                    action.move({0,-100});
                }
            }
            else
            {
                action.move({vTrans.x,-100});
            }
        }
        else
        {
            action.move({vTrans.x, -100});
        }
    }
    else
    {
        action.move(vTrans);
    }

    return false;
}

bool Sheet::setPositionToRelease()
{
    Vector2 devPos = setPoint_[ZONE][3] - odom.getNow();

    Vector2 vTrans =
    {
        constrain(fabs(devPos.x)*1, 100, 500)*setSign(devPos.x, 10),
        100*setSign(devPos.y, 10),
    };

    if(fabs(devPos.x)<=50)
    {
        if(!digitalRead(PIN_LIMIT_CHASSIS_GUIDE_2))
        {
            if(fabs(devPos.x)<=10)
            {
                if(action.isYawDeviationAllowable())
                {
                    chassis.brake();
                    odom.correctY(correctPoint_[ZONE][1].y);
                    return true;
                }
                else
                {
                    action.move({0,-100});
                }
            }
            else
            {
                action.move({vTrans.x,-100});
            }
        }
        else
        {
            action.move({vTrans.x,-100});
        }
    }
    else
    {
        action.move(vTrans);
    }

    return false;
}

void Sheet::initialize()
{
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_1, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_GUIDE_2, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_TOUCH_X, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_TOUCH_Y, INPUT);
    pinMode(PIN_LIMIT_CHASSIS_DELIVER, INPUT);
    pinMode(PIN_LIMIT_DELIVERY_EXTEND, INPUT);
    pinMode(PIN_LIMIT_DELIVERY_SHRINK, INPUT);
    pinMode(PIN_LIMIT_DELIVERY_TOUCH, INPUT);
    pinMode(PIN_LIMIT_ROGER_BOTTOM, INPUT);
}

void Sheet::reset()
{
    stopRoger();
    stopSlider();
    freeHook();
    freeSheet();
    freeYamato();
    pinchDeliverySheet();
}

void Sheet::start()
{
    action.setYawTarget(180);
    tshirt.gripHanger();
    gripSheet();
    shrinkHook();

    if(base.isOnTest())
    {
        odom.correct(setPoint_[ZONE][1]);
        flag_ = FLAG::CORRECT;
        //flag_ = FLAG::SET_2;
        return;
    }

    if(FROM_SZ)
    {
        odom.correct({ZONE_RED ? 3600 : -3600, 0});
        flag_ = FLAG::GO_SZ;
    }
    else
    {
        flag_ = FLAG::GO;
    }
}

void Sheet::debug()
{
    lcd.print(0x10, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_1));
    lcd.print(0x11, digitalRead(PIN_LIMIT_CHASSIS_GUIDE_2));
    lcd.print(0x12, digitalRead(PIN_LIMIT_CHASSIS_TOUCH_X));
    lcd.print(0x13, digitalRead(PIN_LIMIT_CHASSIS_TOUCH_Y));
    lcd.print(0x14, digitalRead(PIN_LIMIT_CHASSIS_DELIVER));
    lcd.print(0x15, digitalRead(PIN_LIMIT_DELIVERY_EXTEND));
    lcd.print(0x16, digitalRead(PIN_LIMIT_DELIVERY_SHRINK));
    lcd.print(0x17, digitalRead(PIN_LIMIT_DELIVERY_TOUCH));
    lcd.print(0x18, digitalRead(PIN_LIMIT_ROGER_BOTTOM), 4);
    lcd.print(0x0d, rogerEncoder_.getCount()/10, 3);
}

bool Sheet::adjustRoger(int count)
{
    if(!isRogerZeroDetected_)
    {
        lowerRoger();
        return false;
    }

    int dev = count - rogerEncoder_.getCount();

    if(abs(dev)<=100)
    {
        stopRoger();
        return true;
    }
    else
    {
        if(dev>0)
        {
            rogerMotor_.setPWM(constrain(-dev*0.04, -50, -20));
        }
        else
        {
            rogerMotor_.setPWM(20);
        }
    }

    return false;
}

bool Sheet::lowerRoger()
{
    if(!digitalRead(PIN_LIMIT_ROGER_BOTTOM))
    {
        stopRoger();
        rogerEncoder_.reset();
        isRogerZeroDetected_ = true;
        return true;
    }
    else
    {
        rogerMotor_.setPWM(30);
        return false;
    }
}

void Sheet::stopRoger()
{
    rogerMotor_.brake();
}

bool Sheet::extendSlider()
{
    if(digitalRead(PIN_LIMIT_DELIVERY_EXTEND))
    {
        stopSlider();
        return true;
    }
    else
    {
        sliderMotor_.setPWM(20);
        return false;
    }
}

bool Sheet::shrinkSlider()
{
    if(digitalRead(PIN_LIMIT_DELIVERY_SHRINK) || digitalRead(PIN_LIMIT_DELIVERY_TOUCH))
    {
        stopSlider();
        return true;
    }
    else
    {
        sliderMotor_.setPWM(-15);
        return false;
    }
}

void Sheet::stopSlider()
{
    sliderMotor_.brake();
}

void Sheet::extendHook()
{
    hookSolenoid_.open();
}

void Sheet::shrinkHook()
{
    hookSolenoid_.close();
}

void Sheet::freeHook()
{
    hookSolenoid_.free();
}

void Sheet::gripSheet1()
{
    gripper1Solenoid_.open();
}

void Sheet::releaseSheet1()
{
    gripper1Solenoid_.close();
}

void Sheet::freeSheet1()
{
    gripper1Solenoid_.free();
}

void Sheet::gripSheet2()
{
    gripper2Solenoid_.open();
}

void Sheet::releaseSheet2()
{
    gripper2Solenoid_.close();
}

void Sheet::freeSheet2()
{
    gripper2Solenoid_.free();
}

void Sheet::gripUpperSheet1()
{
    gripperU1Solenoid_.open();
}

void Sheet::releaseUpperSheet1()
{
    gripperU1Solenoid_.close();
}

void Sheet::freeUpperSheet1()
{
    gripperU1Solenoid_.free();
}

void Sheet::gripUpperSheet2()
{
    gripperU2Solenoid_.open();
}

void Sheet::releaseUpperSheet2()
{
    gripperU2Solenoid_.close();
}

void Sheet::freeUpperSheet2()
{
    gripperU2Solenoid_.free();
}

void Sheet::gripUpperSheet()
{
    gripUpperSheet1();
    gripUpperSheet2();
}

void Sheet::releaseUpperSheet()
{
    releaseUpperSheet1();
    releaseUpperSheet2();
}

void Sheet::freeUpperSheet()
{
    freeUpperSheet1();
    freeUpperSheet2();
}

void Sheet::gripSheet()
{
    gripSheet1();
    gripSheet2();
    gripUpperSheet();
}

void Sheet::releaseSheet()
{
    releaseSheet1();
    releaseSheet2();
    releaseUpperSheet();
}

void Sheet::freeSheet()
{
    freeSheet1();
    freeSheet2();
    freeUpperSheet();
}

void Sheet::extendYamato()
{
    yamatoSolenoid_.open();
}

void Sheet::freeYamato()
{
    yamatoSolenoid_.free();
}

void Sheet::releaseDeliverySheet()
{
    deliverySolenoid_.open();
}

void Sheet::pinchDeliverySheet()
{
    deliverySolenoid_.close();
}

Sheet sheet;