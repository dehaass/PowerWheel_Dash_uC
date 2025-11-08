#pragma once
#include <Arduino.h>

// struct SensorData {
struct __attribute__((packed)) StateData { 
    float batteryVoltage;
    float batteryCurrent;
    float batteryPower;
    int8_t gearPosition; // 0 = Reverse, 1 = Low, 2 = High
    uint8_t throttleStatus; // 0 = off, 1 = on
    uint8_t powerToMotor; // current PWM value, 0 - 255
    uint8_t powerLimit; // maximum allowable PWM value, 0 - 255
    int8_t AccelState; // -1 = decreasing, 0 = steady, 1 = increasing, 2 = stopped
    uint8_t closedLoopControl; // 0 = open loop, 1 = closed loop
};

// struct CommandData {
struct __attribute__((packed)) CommandData {
    uint8_t reset; // 0 = no action, 1 = reset device
    uint8_t killSwitch; // 0 = no action, 1 = kill switch activated
    uint8_t setMaxPower; // 0 - 255
};

template <typename T>
inline uint8_t computeChecksum(const T &d) {
    const uint8_t *ptr = (const uint8_t *)&d;
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(T); i++) sum += ptr[i];
    return sum;
}