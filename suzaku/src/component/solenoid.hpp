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

    static UARTClass& serialSVC_;

    char id_, idNC_, idNO_;

    bool isMulti_;
};