#include "i2c_lcd.hpp"

I2CLCD::I2CLCD(uint8_t contrast, bool is5V, TwoWire* w) : 
lcd_(contrast, is5V)
{
    lcd_.setWire(w);
}

void I2CLCD::initialize()
{
    lcd_.begin(16, 2);
}

void I2CLCD::print(uint8_t position, String str)
{
    if (enabled_)
    {
        uint8_t col = position & 0x0f;
        uint8_t row = (position & 0x10) >> 4;
        lcd_.setCursor(col, row);
        lcd_.print(str);
    }
}

void I2CLCD::print(uint8_t position, String str, int length)
{
    if (enabled_)
    {
        unsigned int strLen = str.length();
        if (strLen > length)
        {
            str = str.substring(0, length - 1);
        }
        else
        {
            for (int i = 0; i < length - strLen; i++)
            {
                str += " ";
            }
        }
        print(position, str);
    }
}

void I2CLCD::print(uint8_t position, int num)
{
    if (enabled_)
    {
        print(position, (String)num);
    }
}

void I2CLCD::print(uint8_t position, int num, int length)
{
    if (enabled_)
    {
        print(position, (String)num, length);
    }
}

void I2CLCD::clear(uint8_t position, int width)
{
    if (enabled_)
    {
        String str = "";
        for (int i = 0; i < width; i++)
        {
            str += " ";
        }
        print(position, str);
    }
}

void I2CLCD::clearAll()
{
    if (enabled_)
    {
        lcd_.clear();
    }
}

void I2CLCD::clearLine(uint8_t row)
{
    if (enabled_)
    {
        print(row & 0x10, "                ");
    }
}

I2CLCD LCD(32, (bool)false, &Wire1);