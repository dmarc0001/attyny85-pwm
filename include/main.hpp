#pragma once
#include <Arduino.h>

constexpr uint8_t ENCODER_A = PB3;
constexpr uint8_t ENCODER_B = PB4;
constexpr uint8_t ENCODER_SW = PB2;
constexpr uint8_t PWM_LED = PB1;
constexpr int32_t MAX_VAL = 255;
constexpr int32_t MIN_VAL = 0;
constexpr int START_BRIGHTNESS = 32;
constexpr int LED_VALUE_ADDR = 2;
constexpr unsigned long DEBOUNCE_MS = 80UL;

// void blink(uint8_t);
void interruptUseSwitch();
void sleepNow();
void wakeUpNow();
