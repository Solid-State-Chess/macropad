#pragma once

#include <stdbool.h>
#include "stm32f042x6.h"

#define CAPSENSE_LEN (5)

/// All state required to measure the rise and fall period of an ADC channel
typedef struct {
    /// GPIO register number used to build offsets when initializing GPIO pin for analog input
    uint8_t gpion;
    /// Last four measured pulse times
    volatile uint16_t time;
    // Set if the sensor is in the rise measurement phase
    volatile bool rise;
    volatile uint32_t count;
} capsense_t;

extern capsense_t SENSORS[CAPSENSE_LEN];

/// Initialize ADC1 to measure capacitive touch channels and TIM16 for timing pulse width.
/// This function must be called prior to creating any capacitive touch structures
void capsense_init(void);

/// Initialize the given GPIO pin as an analog input channel and configure
/// its output mode
void capsense_new(uint8_t gpion[CAPSENSE_LEN]);

/// Take one rise / fall time measurement
void capsense_step(void);

/// Take a pulse measurement from the given sensor (output is in microseconds)
uint16_t capsense_measure(capsense_t *sensor);
