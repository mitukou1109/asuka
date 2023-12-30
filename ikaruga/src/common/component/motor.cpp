#include "motor.hpp"

USARTClass& Motor::serialMDC_ = Serial1;

Motor::Motor(char id) :
    id_(id)
{}

void Motor::initialize()
{
    serialMDC_.begin(115200);
}

void Motor::drive(int velocity)
{
    int direction = velocity>0 ? 1 : 0;
    int velocityData = abs(velocity/2);

    uint8_t data[4] =
    {
        (id_<<5)|(0<<3)|(direction<<2)|((velocityData&(0b011<<9))>>9),
        (id_<<5)|(1<<3)|       ((velocityData&(0b111<<6))>>6)        ,
        (id_<<5)|(2<<3)|       ((velocityData&(0b111<<3))>>3)        ,
        (id_<<5)|(3<<3)|       ((velocityData&(0b111<<0))>>0)
    };
    
    serialMDC_.write(data, 4);
    serialMDC_.flush();
}

void Motor::setPWM(int pwm)
{
    int direction = pwm>0 ? 1 : 0;
    int pwmData = constrain(abs(pwm), 0, 100);

    uint8_t data[4] =
    {
        (id_<<5)|(0<<3)|     (direction<<2)      ,
        (id_<<5)|(1<<3)|((pwmData&(0b001<<6))>>6),
        (id_<<5)|(2<<3)|((pwmData&(0b111<<3))>>3),
        (id_<<5)|(3<<3)|((pwmData&(0b111<<0))>>0)
    };

    serialMDC_.write(data, 4);
    serialMDC_.flush();
}

void Motor::brake()
{
    drive(0);
}

RelayMotor::RelayMotor(char pinCCW, char pinCW) :
    pinCCW_(pinCCW),
    pinCW_(pinCW)
{
    pinMode(pinCCW_, OUTPUT);
    pinMode(pinCW_, OUTPUT);
    digitalWrite(pinCCW_, LOW);
    digitalWrite(pinCW_, LOW);
}

void RelayMotor::drive(char direction)
{
    if(direction == MOTOR_DIRECTION::BRAKE)
    {
        digitalWrite(pinCCW_, LOW);
        digitalWrite(pinCW_, LOW);
    }
    else if(direction == MOTOR_DIRECTION::CCW)
    {
        digitalWrite(pinCCW_, HIGH);
        digitalWrite(pinCW_, LOW);
    }
    else if(direction == MOTOR_DIRECTION::CW)       
    {
        digitalWrite(pinCCW_, LOW);
        digitalWrite(pinCW_, HIGH);
    }
}

void RelayMotor::brake()
{
    drive(MOTOR_DIRECTION::BRAKE);
}