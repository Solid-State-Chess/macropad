#include "sense.hpp"
#include "stm32f042x6.h"
#include "stm32f0xx.h"
#include <stdbool.h>

CapacitiveTouch::CapacitiveTouch(uint8_t gpio_pin) : _gpion{gpio_pin} {
    WRITE_REG(GPIOA->BRR, (1 << _gpion));

    const uint8_t dshift = gpio_pin << 1;

    CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER0_Msk << dshift));

    MODIFY_REG(GPIOA->PUPDR, (GPIO_PUPDR_PUPDR0 << dshift), (GPIO_PUPDR_PUPDR0_1 << dshift));
}

void CapacitiveTouch::calibrate() {
    for(unsigned i = 0; i < NSAMPLE; ++i) {
        add_sample(rise_time());
    }
}

void CapacitiveTouchArray::init_tim16() {
    // Enable TIM16 clock
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_TIM16EN);
    // Ensure timer is disabled before changing timing values
    CLEAR_BIT(TIM16->CR1, TIM_CR1_CEN);
    // Set prescaler for ~0.1us counter resolution
    TIM16->PSC = 0;
    TIM16->RCR = 1;
    // Set auto-reset to maximum counter value - we reset counter manually
    TIM16->ARR = 0xFFFF;
    // Force a timer update event
    SET_BIT(TIM16->EGR, TIM_EGR_UG);
    // Enable the counter again
    SET_BIT(TIM16->CR1, TIM_CR1_CEN);
}

CapacitiveTouchArray::CapacitiveTouchArray(uint8_t tp1, uint8_t tp2, uint8_t tp3, uint8_t tp4, uint8_t tp5) :
_array{
        CapacitiveTouch(tp1),
        CapacitiveTouch(tp2),
        CapacitiveTouch(tp3),
        CapacitiveTouch(tp4),
        CapacitiveTouch(tp5)
    }{
    init_tim16();

    for(unsigned i = 0; i < CAPSENSE_LEN; ++i) {
        _array[i].calibrate();
    }
}

bool CapacitiveTouchArray::step(uint8_t *idx) {
    if(_array[_scan].measure()) {
        *idx = _scan;
        return true;
    } 
    _scan += 1;
    if(_scan >= CAPSENSE_LEN) {
        _scan = 0;
    }

    return false;
}

int16_t CapacitiveTouch::average() {
    uint32_t avg = 0;
    for(unsigned i = 0; i < NSAMPLE; ++i) {
        avg += _samples[i];
    }

    return (int16_t)(avg / NSAMPLE);
}

static inline constexpr int16_t abs(int16_t v) {
    return (v < 0) ? -v : v;
}

bool CapacitiveTouch::measure() {
    int16_t rise = rise_time();
    if(abs(rise - average()) > 15) {
        if(!_pressed) {
            _pressed = true;
            return true;
        }
    } else {
        _pressed = false;
        add_sample(rise);
    }

    return false;
}

void CapacitiveTouch::add_sample(uint16_t sample) {
    for(unsigned i = 0; i < NSAMPLE - 1; ++i) {
        _samples[i] = _samples[i + 1];
    }

    _samples[NSAMPLE - 1] = sample;
}

uint16_t CapacitiveTouch::rise_time(void) {
    uint8_t shift = _gpion << 1;

    // Reset the pulse width timer
    TIM16->CNT = 0;
    
    uint32_t mask = GPIO_PUPDR_PUPDR0_Msk << shift;
    // Enable pullup resistor on the sensor's GPIO pad
    MODIFY_REG(
        GPIOA->PUPDR,
        mask,
        GPIO_PUPDR_PUPDR0_0 << shift
    );
    

    while(READ_BIT(GPIOA->IDR, GPIO_IDR_0 << _gpion) == 0);
    
    //Enable pulldown register
    MODIFY_REG(
        GPIOA->PUPDR,
        mask,
        GPIO_PUPDR_PUPDR0_1 << shift
    );

    while(READ_BIT(GPIOA->IDR, GPIO_IDR_0 << _gpion) != 0);

    return TIM16->CNT;
}
