#pragma once

#include <Arduino.h>

class Solenoid
{
public:

    Solenoid(char);

    Solenoid(char, char);

    static void initialize();

    void open();

    void close();

    void free();

private:

    static void on(char);

    static void off(char);

    static USARTClass& serialSVC_;

    char id_, idNC_, idNO_;

    bool isMulti_;

    bool isInit_ = true;

    bool isOpened_ = false;

    bool isExhausted_ = false;
};