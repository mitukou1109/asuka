#include "odometry.hpp"
#include <Arduino.h>
#include "definition.hpp"
#include "i2c_lcd.hpp"

void raiseRefreshFlag()
{
    odom.refreshFlag_ = true;
}

Odometry::Odometry() :
    odometer({{9, 8}, {11, 10}}),
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
    double yawRet = yaw_+yawBias_;
    if (yawRet > 180)
    {
        yawRet -= 360;
    }
    else if (yawRet < -180)
    {
        yawRet += 360;
    }
    return yawRet;
}

void Odometry::setYawBias(double yawBias)
{
    yawBias_ = yawBias;
}

void Odometry::initialize()
{
    serialGyro_.begin(38400);
    pinMode(PIN_RESET_GYRO, INPUT);
    refreshTimer_.attachInterrupt(raiseRefreshFlag).setPeriod(REFRESH_RATE*1000).start();
}

bool Odometry::refresh()
{
    double delta[2];
    long encoderCount[2];
    static long encoderCountPrev[2];

    if(not refreshFlag_)
    {
        return false;
    }

    if(serialGyro_.available() > 0)
    {
        receiveGyroDataFlag_ = true;
        readYaw();
    }
    else
    {
        receiveGyroDataFlag_ = false;
    }
    
    noInterrupts();
    for(int i = 0; i < 2; i++)
    {
        encoderCount[i] = odometer[i].getCount();
    }
    interrupts();

    for(int i = 0; i < 2; i++)
    {
        delta[i] = (encoderCount[i] - encoderCountPrev[i]) * ODOMETER_C / ENCODER_CPR;
        encoderCountPrev[i] = encoderCount[i];
    }
    //delta[X] *= -1; // -----> x+

    double yaw = toRad(getYaw());
    now_.x += delta[AXIS::X]*cos(yaw) - delta[AXIS::Y]*sin(yaw);
    now_.y += delta[AXIS::X]*sin(yaw) + delta[AXIS::Y]*cos(yaw);

    refreshFlag_ = false;
    return true;
}

void Odometry::correct(Vector2 point)
{
    now_ = point;
}

void Odometry::reset()
{
    resetGyro();
    now_.x = now_.y = 0;
}

void Odometry::resetGyro()
{
    pinMode(PIN_RESET_GYRO, OUTPUT);
    digitalWrite(PIN_RESET_GYRO, LOW);
    pinMode(PIN_RESET_GYRO, INPUT);
    yaw_ = yawBias_ = 0;
}

bool Odometry::canReceiveGyroData()
{
    return receiveGyroDataFlag_;
}

void Odometry::readYaw()
{
    char state = 0, index = 0;
    uint8_t buffer[24] = {};
    timeoutTimer_.reset();

    while(1)
    {
        if(serialGyro_.available()>0)
        {
            uint8_t data = serialGyro_.read();
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
                    int sum = 0;
                    for(int i = 0; i < 23; i++)
                    {
                        sum += buffer[i];
                    }
                    if((sum & 0xff) == buffer[23])
                    {
                        short val = (buffer[6]<<8) | buffer[5];
                        yaw_ = val*0.01;
                        yaw_ *= -1; //CCW+
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

void Odometry::printData()
{
    LCD.print(0x00, now_.x, 5);
    LCD.print(0x06, now_.y, 5);
    LCD.print(0x0c, getYaw(), 4);
}

Odometry odom;