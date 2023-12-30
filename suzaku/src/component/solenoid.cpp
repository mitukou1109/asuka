#include "solenoid.hpp"

UARTClass& Solenoid::serialSVC_ = Serial3;

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
    if(isMulti_)
    {
        on(idNC_);
        off(idNO_);
    }
    else
    {
        on(id_);
    }
}

void Solenoid::close()
{
    if(isMulti_)
    {
        off(idNC_);
        on(idNO_);
    }
    else
    {
        off(id_);
    }
}

void Solenoid::free()
{
    if(isMulti_)
    {
        off(idNC_);
        off(idNO_);
    }
    else
    {
        off(id_);
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