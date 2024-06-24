#include <stdint.h>
#include "sense.h"
#include "system_stm32f0xx.h"
#include <stm32f0xx.h>
#include <stm32f042x6.h>
#include <core_cm0.h>
#include <tusb.h>

static void init_leds(void) {
    //Reset A14 and A13 output
    WRITE_REG(GPIOA->BRR, GPIO_BRR_BR_14);
    WRITE_REG(GPIOA->BRR, GPIO_BRR_BR_13);
    
    //Set pins to output
    MODIFY_REG(
        GPIOA->MODER,
        GPIO_MODER_MODER14 | GPIO_MODER_MODER13,
        GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0
    );
    
    //Set low speed IO
    CLEAR_BIT(
        GPIOA->OSPEEDR,
        GPIO_OSPEEDR_OSPEEDR13 | GPIO_OSPEEDR_OSPEEDR14
    );
    
    //Enable pulldown resistors
    MODIFY_REG(
        GPIOA->PUPDR,
        GPIO_PUPDR_PUPDR13 | GPIO_PUPDR_PUPDR14,
        GPIO_PUPDR_PUPDR13_1 | GPIO_PUPDR_PUPDR14_1
    );
}


int main(void) {
    tud_init(1);
    //NVIC_EnableIRQ(TIM16_IRQn);
    //__NVIC_SetPriority(TIM16_IRQn, 0);
    
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);

    SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY);

    //Enable HSI clock and wait for ready
    SET_BIT(RCC->CR, RCC_CR_HSION);
    while(READ_BIT(RCC->CR, RCC_CR_HSIRDY) != RCC_CR_HSIRDY);
    //Set HSI calibration trim to 16
    SET_BIT(RCC->CR, RCC_CR_HSICAL_4);
    
    //Set AHB and APB1 prescalers to 1
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE, RCC_CFGR_PPRE_DIV1);
    
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
    
    //Enable GPIO port clock for GPIOA
    SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
    

    capsense_init();
    init_leds();
    capsense_new((uint8_t[]){1, 0, 5, 6, 7});
    
    uint32_t ios[2] = {GPIO_ODR_13, GPIO_ODR_14};
	for(;;) {
        tud_task();
        for(uint8_t i = 0; i < CAPSENSE_LEN; ++i) {
            capsense_step();
        }

        for(uint8_t i = CAPSENSE_LEN; i > 0; --i) {
            if(SENSORS[i - 1].time > 65) {
                SET_BIT(GPIOA->ODR, ios[i & 1]);
            } else {
                CLEAR_BIT(GPIOA->ODR, ios[i & 1]);
            }
        }

    }
}

// Invoked when device is mounted
void tud_mount_cb(void) {
    SET_BIT(GPIOA->ODR, GPIO_ODR_14);
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    CLEAR_BIT(GPIOA->ODR, GPIO_ODR_14);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
  // This example doesn't use multiple report and report ID
  (void) itf;
  (void) report_id;
  (void) report_type;

  // echo back anything we received from host
  tud_hid_report(0, buffer, bufsize);
}