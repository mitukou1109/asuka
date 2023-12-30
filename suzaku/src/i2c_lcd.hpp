#pragma once

#include <I2CLiquidCrystal.h>

class I2CLCD
{
public:

    I2CLCD(uint8_t, bool, TwoWire*);

    void initialize();

    void print(uint8_t, String);

    void print(uint8_t, String, int);

    void print(uint8_t, int);

    void print(uint8_t, int, int);

    void clear(uint8_t, int = 1);

    void clearAll();
    
    void clearLine(uint8_t);

private:

    I2CLiquidCrystal lcd_;
};

extern I2CLCD lcd;