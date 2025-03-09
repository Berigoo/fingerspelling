#include "button.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "hal/gpio_types.h"
#include "soc/gpio_num.h"
#include <cstdint>

// 18 and 19
static const uint64_t btn_mask = (1ULL << GPIO_NUM_18) | (1ULL << GPIO_NUM_19);

void btn_setup() { gpio_install_isr_service(0); }

//TODO non ISR INTR
void btn_init() {
  gpio_config_t conf = {};
  
  conf.intr_type = GPIO_INTR_POSEDGE;
  conf.mode = GPIO_MODE_INPUT;
  conf.pin_bit_mask = btn_mask;
  conf.pull_down_en = GPIO_PULLDOWN_ENABLE;

  ESP_ERROR_CHECK(gpio_config(&conf));
}

void btn_attach_isr(gpio_num_t gpio_num, gpio_isr_t gpio_isr_handler,
                    void *args) {
  // ESP_ERROR_CHECK(gpio_isr_register(gpio_isr_handler, nullptr,
  //                                   0, nullptr));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_num, gpio_isr_handler, args));
}
void btn_clean(gpio_num_t gpio_num) {
  ESP_ERROR_CHECK(gpio_isr_handler_remove(gpio_num));
}
