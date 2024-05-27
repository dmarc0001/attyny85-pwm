#pragma once
#include <Arduino.h>

//
// constants fpr programm
//
constexpr uint8_t ENCODER_A = PB3;          //! rotary encoder A
constexpr uint8_t ENCODER_B = PB4;          //! rotary encoder B
constexpr uint8_t ENCODER_SW = PB2;         //! rotary encoder push button
constexpr uint8_t PWM_LED_1 = PB1;          //! LED PWM output
constexpr uint8_t PWM_LED_0 = PB2;          //! LED PWM output
constexpr int32_t MAX_VAL = 255;            //! PWM Max value
constexpr int32_t MIN_VAL = 0;              //! PWM min value
constexpr int32_t MIN_VAL_AFTER_SLEEP = 5;  //! Min Value after controller sleep
constexpr int32_t START_DOUBLE_SPEED = 16;  //! over for double speed
constexpr int START_BRIGHTNESS = 16;        //! initial brightness
constexpr int LED_VALUE_ADDR = 1;
constexpr int EEPROM_INIT_MAGIC = 0xaa;
constexpr unsigned long DEBOUNCE_MS = 20UL;

//
// prototypes for functions
//
void initEEPROM();
void interruptSwitch();
void interruptSwitchWakeup();
void sleepNow();
void wakeUpNow();
bool debounceSwitch( uint8_t );
