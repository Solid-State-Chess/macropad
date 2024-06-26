#include "class/hid/hid.h"
#include "tusb_config.h"
#include <stdint.h>
#include "device/usbd.h"
#include "sense.hpp"
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

static void blink(void) {
    for(;;) {
        GPIOA->ODR ^= GPIO_ODR_14;
        for(unsigned i = 0; i < 250000; ++i) {
            READ_BIT(GPIOA->ODR, GPIO_ODR_14);
        }
    }
}

int main(void) {
    //NVIC_EnableIRQ(TIM16_IRQn);
    //__NVIC_SetPriority(USB_IRQn, 100);
    
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

    init_leds(); 

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_CRSEN);
    
    SET_BIT(RCC->CR2, RCC_CR2_HSI48ON);
    while(READ_BIT(RCC->CR2, RCC_CR2_HSI48RDY) != RCC_CR2_HSI48RDY);

    SET_BIT(CRS->CR, CRS_CR_AUTOTRIMEN);
    MODIFY_REG(CRS->CFGR, CRS_CFGR_SYNCSRC_Msk, CRS_CFGR_SYNCSRC_1);
    SET_BIT(SYSCFG->CFGR1, SYSCFG_CFGR1_PA11_PA12_RMP);

    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USBEN);

    if(!tusb_init()) {
        blink();
    }

    
    CapacitiveTouchArray sensor{0, 1, 5, 6, 7};

    bool pending = false;
    bool report = true;

    static uint8_t KEYS[CAPSENSE_LEN] = {
        HID_KEY_ENTER,
        HID_KEY_6,
        HID_KEY_1,
        HID_KEY_F,
        HID_KEY_L
    };

    static uint8_t modifications[CAPSENSE_LEN] = {
        0,
        0,
        0,
        0,
        KEYBOARD_MODIFIER_LEFTGUI | KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTALT
    };

    uint8_t mod = 0;

    static uint8_t keycode[6] = {0};
	for(;;) {


        uint8_t pressed = 0;
        if(sensor.step(&pressed) && !pending) {
            mod = modifications[pressed];
            keycode[0] = KEYS[pressed];
            pending = true;
        } else {

        }

        if(pending && tud_hid_ready()) {
            pending = false;
            report = true;
            tud_hid_keyboard_report(1, mod, keycode);
        }
        
        if(report) {
            if(tud_hid_ready()) {
                report = false;
                keycode[0] = 0;
                tud_hid_keyboard_report(1, 0, keycode);
            }
        }

        tud_task();
    }
}

extern "C" void USB_IRQHandler() {
  tud_int_handler(0);
}

// Invoked when device is mounted
void tud_mount_cb(void) {
    //SET_BIT(GPIOA->ODR, GPIO_ODR_14);
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    //CLEAR_BIT(GPIOA->ODR, GPIO_ODR_14);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    //SET_BIT(GPIOA->ODR, GPIO_ODR_14);
    (void)remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    //SET_BIT(GPIOA->ODR, GPIO_ODR_14);
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
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
}
