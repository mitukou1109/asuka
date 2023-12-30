#include "encoder.hpp"
#include <Arduino.h>

char Encoder::pin_[8][2] = {};
volatile long Encoder::count_[8] = {};
char Encoder::numberOfEncoders = 0;

void readEncoder0()
{
    Encoder::readEncoder(0);
}

void readEncoder1()
{
    Encoder::readEncoder(1);
}

void readEncoder2()
{
    Encoder::readEncoder(2);
}

void readEncoder3()
{
    Encoder::readEncoder(3);
}

void readEncoder4()
{
    Encoder::readEncoder(4);
}

void readEncoder5()
{
    Encoder::readEncoder(5);
}

void readEncoder6()
{
    Encoder::readEncoder(6);
}

void readEncoder7()
{
    Encoder::readEncoder(7);
}

void (*readEncoderN[])() =
{
    readEncoder0,
    readEncoder1,
    readEncoder2,
    readEncoder3,
    readEncoder4,
    readEncoder5,
    readEncoder6,
    readEncoder7
};

Encoder::Encoder(char pinA, char pinB) :
    id_(numberOfEncoders)
{
    ++numberOfEncoders;

    pin_[id_][PHASE::A] = pinA;
    pin_[id_][PHASE::B] = pinB;

    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(pinA, readEncoderN[id_], CHANGE);
    attachInterrupt(pinB, readEncoderN[id_], CHANGE);
}

void Encoder::readEncoder(char id)
{
    static char statePrev[8][2];
    char state[2];

    for (int i=0; i<2; i++)
    {
        state[i] = not digitalRead(pin_[id][i]);
    }

    if (state[A] != statePrev[id][PHASE::A] and state[PHASE::B] != statePrev[id][PHASE::B])
    {
        return;
    }
    else if (state[A] != statePrev[id][PHASE::A])
    {
        if (state[PHASE::A] == state[PHASE::B])
        {
            count_[id]++;
        }
        else if (state[PHASE::A] != state[PHASE::B])
        {
            count_[id]--;
        }
    }
    else if (state[PHASE::B] != statePrev[id][PHASE::B])
    {
        if (state[PHASE::A] == state[PHASE::B])
        {
            count_[id]--;
        }
        else if (state[PHASE::A] != state[PHASE::B])
        {
            count_[id]++;
        }
    }

    for(int i = 0;i < 2; i++)
    {
        statePrev[id][i] = state[i];
    }
}

long Encoder::getCount()
{
    return count_[id_];
}

void Encoder::reset()
{
    count_[id_] = 0;
}