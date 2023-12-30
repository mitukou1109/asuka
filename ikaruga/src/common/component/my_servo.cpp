#include "my_servo.hpp"

PCA9685 MyServo::driver = PCA9685();

MyServo::MyServo(char ch) :
ch_(ch)
{}

void MyServo::initialize()
{
    driver.begin();
    driver.setPWMFreq(60);
}

void MyServo::set(int angle)
{
    uint16_t pulse = map(angle, 0, 180, 500, 150);
    driver.setPWM(ch_, pulse, 0);
}