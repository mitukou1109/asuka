#pragma once

class Encoder
{
    friend void readEncoder0();
    friend void readEncoder1();
    friend void readEncoder2();
    friend void readEncoder3();
    friend void readEncoder4();
    friend void readEncoder5();
    friend void readEncoder6();
    friend void readEncoder7();

public:

    Encoder(char pinA, char pinB);

    long getCount();
    
    void reset();

private:

    static void readEncoder(char);

    enum PHASE
    {
        A,
        B
    };

    static char pin_[8][2];
    static volatile long count_[8];
    static char numberOfEncoders;

    char id_;
};