#include "freertos/FreeRTOS.h"
#include "flex_sensor.h"
#include "button.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "utils.h"

//TODO report flex sensor value to other host

// low pass alpha
static const float alpha = 0.5; // fifty-fifty
static QueueHandle_t intr_queue;

static void IRAM_ATTR btn_isr_handler(void *arg);
static void report_flex_values(void* args);

extern "C"
void app_main(void)
{
  btn_setup();
  btn_init();
  btn_attach_isr(GPIO_NUM_18, btn_isr_handler, nullptr);
  
  flex_init(ADC_CHANNEL_3);

  intr_queue = xQueueCreate(2, 0);
  xTaskCreate(report_flex_values, "report_flex_values", 2048, NULL, 1, NULL);
  
  
  float filteredValue = flex_read(ADC_CHANNEL_3);

  while (1) {
    float val = flex_read(ADC_CHANNEL_3);

    filteredValue = blend(alpha, val, filteredValue);
    float normalized = flex_normalize_voltage(filteredValue);

    ESP_LOGI("MAIN", "\t reading result (voltage):\t %f", filteredValue);    
    ESP_LOGI("MAIN", "\t reading result (normalized):\t %f", normalized);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void btn_isr_handler(void *arg) {
  xQueueSendFromISR(intr_queue, nullptr, nullptr);
//   // float oldVal = flex_read(ADC_CHANNEL_3);
//   // vTaskDelay(pdMS_TO_TICKS(1000)); // for smoothing
  
//   // float val = flex_read(ADC_CHANNEL_3);

//   // oldVal = blend(alpha, val, oldVal);
//   // float normalized = flex_normalize_voltage(oldVal);

//   // ESP_LOGI("MAIN", "\t reading result (voltage):\t %f", oldVal);
//   // ESP_LOGI("MAIN", "\t reading result (normalized):\t %f", normalized);

//   // ESP_LOGI("MAIN", "Hello");
}

void report_flex_values(void *args) {
  while (1) {
    if (xQueueReceive(intr_queue, nullptr, portMAX_DELAY)) {
      float prevVal = flex_read(ADC_CHANNEL_3);
      vTaskDelay(pdMS_TO_TICKS(1000)); // for smoothing
  
      float val = flex_read(ADC_CHANNEL_3);

      prevVal = blend(alpha, val, prevVal);
      float normalized = flex_normalize_voltage(prevVal);

      ESP_LOGI("BTN", "\t reading result (voltage):\t %f", prevVal);
      ESP_LOGI("BTN", "\t reading result (normalized):\t %f", normalized);
    }
  }
}
  
