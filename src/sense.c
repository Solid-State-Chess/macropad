#include "sense.h"
#include "stm32f042x6.h"
#include "stm32f0xx.h"
#include <stdbool.h>

capsense_t SENSORS[CAPSENSE_LEN] = {0};

static volatile capsense_t * volatile measured = 0;


void capsense_new_sgl(capsense_t *sensor, uint8_t gpion);

void capsense_new(uint8_t gpion[CAPSENSE_LEN]) {
    for(uint8_t i = 0; i < CAPSENSE_LEN; ++i) {
        capsense_new_sgl(&SENSORS[i], gpion[i]);
    }
}

void capsense_new_sgl(capsense_t *sensor, uint8_t gpion) {
    //Clear output for the selected pin
    WRITE_REG(GPIOA->BRR, (1 << gpion));

    uint8_t dshift = gpion * 2;
    // Set general purpose input mode
    CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODER0_Msk << dshift));
    // Enable pull down resistor
    MODIFY_REG(GPIOA->PUPDR, (GPIO_PUPDR_PUPDR0 << dshift), (GPIO_PUPDR_PUPDR0_1 << dshift));

    sensor->gpion = gpion;
    sensor->time = 0;
}


static void tim16_init(void) {
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


void capsense_init(void) {
    tim16_init();
}


static uint8_t scan = 0;

void capsense_step(void) {
    capsense_measure(&SENSORS[scan]);
    scan += 1;
    if(scan >= CAPSENSE_LEN) {
        scan = 0;
    }
}

uint16_t capsense_measure(capsense_t *sensor) {
    uint8_t shift = sensor->gpion * 2;

    //Spinlock if there is an ongoing measurement
    while(measured != 0);
    

    measured = sensor;
    measured->rise = true; 

    // Reset the pulse width timer
    TIM16->CNT = 0;
    
    uint32_t mask = GPIO_PUPDR_PUPDR0_Msk << shift;
    // Enable pullup resistor on the sensor's GPIO pad
    MODIFY_REG(
        GPIOA->PUPDR,
        mask,
        GPIO_PUPDR_PUPDR0_0 << shift
    );
    

    while(READ_BIT(GPIOA->IDR, GPIO_IDR_0 << sensor->gpion) == 0);
    
    //Enable pulldown register
    MODIFY_REG(
        GPIOA->PUPDR,
        mask,
        GPIO_PUPDR_PUPDR0_1 << shift
    );

    while(READ_BIT(GPIOA->IDR, GPIO_IDR_0 << sensor->gpion) != 0);

    sensor->time = TIM16->CNT;

    measured = 0;
    
    return sensor->time;
}
