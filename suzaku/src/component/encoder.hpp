#pragma once

#define MAX_NUM_ENCODERS 5

class Encoder
{
    friend void readEncoder0();
    friend void readEncoder1();
    friend void readEncoder2();
    friend void readEncoder3();
    friend void readEncoder4();

public:

    Encoder(char, char);

    long getCount();
    
    void reset();

private:

    enum PHASE
    {
        A = 0,
        B = 1
    };

    static void readEncoder(char);

    static char pin_[MAX_NUM_ENCODERS][2];

    static volatile long count_[MAX_NUM_ENCODERS];

    static char number_;

    char id_;
};