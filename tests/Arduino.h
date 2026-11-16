#pragma once
#include <cstddef>
#include <cstdint>
#include <cmath>

extern uint32_t testMillis;
inline uint32_t millis() { return testMillis; }
inline void delay(uint32_t ms) { testMillis += ms; }
