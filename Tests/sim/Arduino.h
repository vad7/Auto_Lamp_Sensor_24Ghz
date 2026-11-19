// Симулятор на ПК: заглушка Arduino - только то, что берут исходники прошивки вне board_rp2040.cpp.
// Время виртуальное (sim_core.cpp): millis() и delay() двигают часы симулятора, а не ждут.
#pragma once
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint32_t millis();          // виртуальное время, мс; каждый вызов - SIM_CALL_COST_US «работы процессора»
void     delay(uint32_t ms);   // сдвиг виртуального времени

class Print {
public:
    virtual ~Print() {}
    virtual size_t write(uint8_t b) = 0;
    virtual size_t write(const uint8_t* buf, size_t n) {
        for (size_t i = 0; i < n; i++) {
            write(buf[i]);
        }
        return n;
    }
    virtual void flush() {}
    size_t printf(const char* fmt, ...) __attribute__((format(gnu_printf, 2, 3)));
};

class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
};

class SimSerial : public Stream {   // монитор порта USB: вывод в stdout с отметкой времени
public:
    size_t write(uint8_t b);
    int available() { return 0; }
    int read() { return -1; }
};
extern SimSerial Serial;
