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
static void report_flex_values(void *args);
static bool isPause = false;

static void *flex_thumb_context;
static void *flex_index_context;
static void *flex_middle_context;
static void *flex_ring_context;
static void *flex_pinky_context;

extern "C"
void app_main(void)
{
  btn_setup();
  btn_init();
  btn_attach_isr(GPIO_NUM_18, btn_isr_handler, nullptr);

  flex_thumb_context = flex_create(ADC_CHANNEL_0);
  flex_index_context = flex_create(ADC_CHANNEL_1);
  flex_middle_context = flex_create(ADC_CHANNEL_2);
  flex_ring_context = flex_create(ADC_CHANNEL_3);
  flex_pinky_context = flex_create(ADC_CHANNEL_4);  

  intr_queue = xQueueCreate(2, 0);
  xTaskCreate(report_flex_values, "report_flex_values", 2048, NULL, 1, NULL);

  float filtered_value_0 = flex_read(flex_thumb_context);
  float filtered_value_1 = flex_read(flex_index_context);
  float filtered_value_2 = flex_read(flex_middle_context);
  float filtered_value_3 = flex_read(flex_ring_context);
  float filtered_value_4 = flex_read(flex_pinky_context);  

  while (1) {
    float val_0 = flex_read(flex_thumb_context);
    float val_1 = flex_read(flex_index_context);
    float val_2 = flex_read(flex_middle_context);
    float val_3 = flex_read(flex_ring_context);
    float val_4 = flex_read(flex_pinky_context);

    filtered_value_0 = blend(alpha, val_0, filtered_value_0);
    filtered_value_1 = blend(alpha, val_1, filtered_value_1);    
    filtered_value_2 = blend(alpha, val_2, filtered_value_2);
    filtered_value_3 = blend(alpha, val_3, filtered_value_3);
    filtered_value_4 = blend(alpha, val_4, filtered_value_4);

    float normalized_0 = flex_normalize_voltage(filtered_value_0);
    float normalized_1 = flex_normalize_voltage(filtered_value_1);
    float normalized_2 = flex_normalize_voltage(filtered_value_2);
    float normalized_3 = flex_normalize_voltage(filtered_value_3);
    float normalized_4 = flex_normalize_voltage(filtered_value_4);

    ESP_LOGI("MAIN", "\t reading result (normalized):\t %f,%f,%f,%f,%f",
             normalized_0, normalized_1, normalized_2, normalized_3,
             normalized_4);
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void btn_isr_handler(void *arg) {
  xQueueSendFromISR(intr_queue, nullptr, nullptr);
}

void report_flex_values(void *args) {
  while (1) {
    if (xQueueReceive(intr_queue, nullptr, portMAX_DELAY)) {
      isPause = !isPause;

      ESP_LOGI("BTN", "isPause: %d", isPause);
      // float prevVal = flex_read(ADC_CHANNEL_3);
      // vTaskDelay(pdMS_TO_TICKS(1000)); // for smoothing
  
      // float val = flex_read(ADC_CHANNEL_3);

      // prevVal = blend(alpha, val, prevVal);
      // float normalized = flex_normalize_voltage(prevVal);

      // ESP_LOGI("BTN", "\t reading result (voltage):\t %f", prevVal);
      // ESP_LOGI("BTN", "\t reading result (normalized):\t %f", normalized);
    }
  }
}
  
