#include "solenoid.hpp"

USARTClass& Solenoid::serialSVC_ = Serial3;

Solenoid::Solenoid(char id) : 
    id_(id),
    idNC_(NULL),
    idNO_(NULL),
    isMulti_(false)
{}

Solenoid::Solenoid(char idNC, char idNO) : 
    id_(NULL),
    idNC_(idNO),
    idNO_(idNC),
    isMulti_(true)
{}

void Solenoid::initialize()
{
    serialSVC_.begin(38400);
}

void Solenoid::open()
{
    if (isInit_ or not isOpened_ or isExhausted_)
    {
        if (isMulti_)
        {
            on(idNC_);
            off(idNO_);
        }
        else
        {
            on(id_);
        }
        isInit_ = false;
        isOpened_ = true;
        isExhausted_ = false;
    }
}

void Solenoid::close()
{
    if (isInit_ or isOpened_ or isExhausted_)
    {
        if (isMulti_)
        {
            off(idNC_);
            on(idNO_);
        }
        else
        {
            off(id_);
        }
        isInit_ = false;
        isOpened_ = false;
        isExhausted_ = false;
    }
}

void Solenoid::free()
{
    if (isInit_ or not isExhausted_)
    {
        if (isMulti_)
        {
            off(idNC_);
            off(idNO_);
        }
        else
        {
            off(id_);
        }
        isInit_ = false;
        isOpened_ = false;
        isExhausted_ = true;
    }
}

void Solenoid::on(char id)
{
    uint8_t data = (id<<1)|1;
    serialSVC_.write(data);
}

void Solenoid::off(char id)
{
    uint8_t data = (id<<1)|0;
    serialSVC_.write(data);
}