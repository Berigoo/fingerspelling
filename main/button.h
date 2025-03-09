#ifndef MAIN_BUTTON_H
#define MAIN_BUTTON_H

#include "driver/gpio.h"
#include "soc/gpio_num.h"

void btn_setup();
void btn_init();
void btn_attach_isr(gpio_num_t gpio_num, gpio_isr_t gpio_isr_handler, void* args);
void btn_clean(gpio_num_t gpio_num);
//TODO uninstall isr service

#endif // MAIN_BUTTON_H










