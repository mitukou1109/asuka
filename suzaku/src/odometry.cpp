#include "odometry.hpp"
#include <Arduino.h>
#include "definition.hpp"
#include "i2c_lcd.hpp"
#include "base.hpp"

void raiseRefreshFlag()
{
    odom.refreshFlag_ = true;
}

Odometry::Odometry() :
odometer_({ {11,10} , {9,8} }),
serialGyro_(Serial3),
refreshTimer_(0),
timeoutTimer_(1)
{}

Vector2 Odometry::getNow() const
{
    return now_;
}

double Odometry::getYaw() const
{
    double yawRet = yaw_ + yawAdd_;

    if(yawRet>180)
    {
        yawRet -= 360;
    }
    else if(yawRet<-180)
    {
        yawRet += 360;
    }

    return yawRet;
}

void Odometry::initialize()
{
    serialGyro_.begin(38400);
    refreshTimer_.attachInterrupt(raiseRefreshFlag).setPeriod(REFRESH_RATE*1000).start();
}

void Odometry::reset()
{
    now_.x = now_.y = yaw_ = 0;
}

void Odometry::resetGyro()
{
    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, LOW);
    pinMode(PIN_RESET, INPUT);
}

bool Odometry::refresh()
{
    double delta[2];
    long odometerCount[2];
    static long odometerCountPrev[2];

    if(serialGyro_.available()>0)
    {
        base.setLED(2, LOW);
        readGyro();
    }
    else
    {
        if(timeoutTimer_.wait(500))
        {
            base.setLED(2, HIGH);
        }
    }

    if(!refreshFlag_)
    {
        return false;
    }

    noInterrupts();
    for(int i=0;i<2;i++)
    {
        odometerCount[i] = odometer_[i].getCount();
    }
    interrupts();

    for(int i=0;i<2;i++)
    {
        delta[i] = (odometerCount[i] - odometerCountPrev[i]) / ODOMETER_CPR * ODOMETER_C;
        odometerCountPrev[i] = odometerCount[i];
    }
    delta[AXIS::X] *= -1; // -----> x+

    double yaw = toRad(getYaw());
    now_.x += delta[AXIS::X]*cos(yaw) - delta[AXIS::Y]*sin(yaw);
    now_.y += delta[AXIS::X]*sin(yaw) + delta[AXIS::Y]*cos(yaw);

    refreshFlag_ = false;

    return true;
}

void Odometry::correct(Vector2 reference)
{
    now_ = reference;
}

void Odometry::correctX(double reference)
{
    now_.x = reference;
}

void Odometry::correctY(double reference)
{
    now_.y = reference;
}

void Odometry::setYawAdd(double yawAdd)
{
    yawAdd_ = yawAdd;
}

void Odometry::readGyro()
{
    char state = 0, index = 0;
    int sum = 0;
    byte buffer[24] = {};

    timeoutTimer_.reset();

    while(1)
    {
        if(serialGyro_.available()>0)
        {
            byte data = serialGyro_.read();

            switch(state)
            {
                case 0:
                if(data == 0xA6)
                {
                    state++;
                }
                break;

                case 1:
                if(data == 0xA6)
                {
                    state++;
                }
                else
                {
                    state = index = 0;
                }
                break;

                case 2:
                buffer[index++] = data;
                if(index >= 24)
                {
                    state = index = 0;
                    for(int i=0;i<23;i++)
                    {
                        sum += buffer[i];
                    }
                    if((sum & 0xff) == buffer[23])
                    {
                        short val = (buffer[6]<<8)|buffer[5];
                        yaw_ = val*0.01;
                        yaw_ *= -1; //CCW+
                        flushGyro();
                        return;
                    }
                }
                break;
            }
        }

        if(timeoutTimer_.wait(200))
        {
            return;
        }
    }
}

void Odometry::flushGyro()
{
    while(serialGyro_.available()>0)
    {
        serialGyro_.read();
    }
    timeoutTimer_.reset();
}

void Odometry::printData()
{
    lcd.print(0x00, getNow().x, 6);
    lcd.print(0x06, getNow().y, 6);
    lcd.print(0x0c, getYaw(), 4);
}

Odometry odom;