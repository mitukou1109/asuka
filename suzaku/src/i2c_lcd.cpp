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
    uint8_t col = position&0x0f;
    uint8_t row = (position&0x10)>>4;
    lcd_.setCursor(col, row);
    lcd_.print(str);
}

void I2CLCD::print(uint8_t position, String str, int length)
{
    unsigned int strLen = str.length();

    if(strLen > length)
    {
        str = str.substring(0, length-1);
    }
    else
    {
        for(int i=0; i<length-strLen; i++)
        {
            str += " ";
        }
    }

    print(position, str);
}

void I2CLCD::print(uint8_t position, int num)
{
    print(position, (String)num);
}

void I2CLCD::print(uint8_t position, int num, int length)
{
    print(position, (String)num, length);
}

void I2CLCD::clear(uint8_t position, int width)
{
    String str = "";
    for(int i=0;i<width;i++)
    {
        str += " ";
    }
    print(position, str);
}

void I2CLCD::clearAll()
{
    clear(0x00, 16);
    clear(0x10, 16);
}

void I2CLCD::clearLine(uint8_t row)
{
    clear(row&0x10, 16);
}

I2CLCD lcd(36, (bool)false, &Wire1);