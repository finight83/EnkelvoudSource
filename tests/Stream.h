#pragma once
#include "Arduino.h"

class Print {
public:
    virtual ~Print() = default;
    virtual size_t write(const uint8_t *, size_t) = 0;
    virtual size_t write(uint8_t) = 0;
    virtual void flush() {}
};

class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
};
