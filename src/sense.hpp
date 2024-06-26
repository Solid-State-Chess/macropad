#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CAPSENSE_LEN (5)

/// Capacitive touch sensor measuring running average and standard deviation of a timed
/// touch pad
struct CapacitiveTouch {
public:
    static inline constexpr const uint8_t NSAMPLE = 16;

    CapacitiveTouch(uint8_t gpio_pin);

    bool measure(void);

    int16_t average();

    void calibrate();
private:
    /// Add a sample to the samples buffer without checking for variance
    void add_sample(uint16_t sample);

    uint16_t rise_time();

    const uint8_t _gpion;
    bool _pressed{false};
    uint16_t _samples[NSAMPLE];
};


struct CapacitiveTouchArray {
public:
    CapacitiveTouchArray(uint8_t tp1, uint8_t tp2, uint8_t tp3, uint8_t tp4, uint8_t tp5);
    
    /// Step the sensor once, measuring the rise time of a single sensor
    /// \param [out] idx Set to the index of a pressed touch pad if one is pressed
    /// \return true if a new pressed event was detected
    bool step(uint8_t *idx);
private:
    static void init_tim16(void);

    CapacitiveTouch _array[CAPSENSE_LEN];
    uint8_t _scan{0};
};
