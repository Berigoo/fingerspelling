#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "flex_sensor.h"
#include "button.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "training_data.h"
#include "utils.h"
#include "mpu60x0.h"
#include "soft_i2c_master.h"
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>
#include "tf.h"

//TODO report flex sensor value to other host

static const float a = 0.5; // for basic linear interpolation
static QueueHandle_t intr_queue;

// static void IRAM_ATTR btn_isr_handler(void *arg);
// static void report_flex_values(void *args);
static bool isPause = false;

static i2c_master_bus_handle_t s_i2cHandle1;
static i2c_master_bus_handle_t s_i2cHandle2;
static soft_i2c_master_bus_t s_i2cHandle3;
static void *s_adcUnitContext;
static void *s_adcUnitContext2;

static void *s_flexThumbContext;
static void *s_flexIndexContext;
static void *s_flexMiddleContext;
static void *s_flexRingContext;
static void *s_flexPinkyContext;

extern "C" void app_main(void) {
  tfSetup();
  // btn_setup();
  // btn_init();
  // btn_attach_isr(GPIO_NUM_18, btn_isr_handler, nullptr);
  {
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.scl_io_num = GPIO_NUM_4; // TODO kconfig
    i2c_mst_config.sda_io_num = GPIO_NUM_5;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &s_i2cHandle1));
  }
  {
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_1;
    i2c_mst_config.scl_io_num = GPIO_NUM_9; // TODO kconfig
    i2c_mst_config.sda_io_num = GPIO_NUM_10;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &s_i2cHandle2));
  }
  {
    soft_i2c_master_config_t conf{};
    conf.scl_pin = GPIO_NUM_12;
    conf.sda_pin = GPIO_NUM_13;
    conf.freq = SOFT_I2C_200KHZ;

    ESP_ERROR_CHECK(soft_i2c_master_new(&conf, &s_i2cHandle3));
  }

  s_adcUnitContext = adc_unit_create(ADC_UNIT_1);
  s_adcUnitContext2 = adc_unit_create(ADC_UNIT_2);

  Mpu mpuBack(s_i2cHandle2, 400000, false, 512.0f, 0.9f);
  Mpu mpuIndex(s_i2cHandle1, 400000, false, 512.0f, 0.9f);
  Mpu mpuMiddle(s_i2cHandle1, 400000, true, 512.0f, 0.9f);
  Mpu mpuRing(s_i2cHandle2, 400000, true, 512.0f, 0.9);
  Mpu mpuPinky(s_i2cHandle3, false, 512.0f, 0.9f);

  s_flexThumbContext = flex_create(s_adcUnitContext, ADC_CHANNEL_5); // GPIO 6
  s_flexIndexContext = flex_create(s_adcUnitContext2, ADC_CHANNEL_9); // GPIO 20
  s_flexMiddleContext = flex_create(s_adcUnitContext, ADC_CHANNEL_7); // GPIO 8
  s_flexRingContext = flex_create(s_adcUnitContext2, ADC_CHANNEL_6); // GPIO 17
  s_flexPinkyContext = flex_create(s_adcUnitContext2, ADC_CHANNEL_7); // GPIO 18

  // // intr_queue = xQueueCreate(2, 0);
  // // xTaskCreate(report_flex_values, "report_flex_values", 2048, NULL, 1, NULL);

  float filtered_value_0 = flex_read(s_flexThumbContext);
  float filtered_value_1 = flex_read(s_flexIndexContext);
  float filtered_value_2 = flex_read(s_flexMiddleContext);
  float filtered_value_3 = flex_read(s_flexRingContext);
  float filtered_value_4 = flex_read(s_flexPinkyContext);

  size_t currTime = esp_timer_get_time();
  const size_t updateTime = 2000000;
  while (1) {
    mpuBack.update();
    mpuIndex.update();
    mpuMiddle.update();
    mpuRing.update();
    mpuPinky.update();
    
    float val_0 = flex_read(s_flexThumbContext);
    float val_1 = flex_read(s_flexIndexContext);
    float val_2 = flex_read(s_flexMiddleContext);
    float val_3 = flex_read(s_flexRingContext);
    float val_4 = flex_read(s_flexPinkyContext);

    filtered_value_0 = blend(a, val_0, filtered_value_0);
    filtered_value_1 = blend(a, val_1, filtered_value_1);    
    filtered_value_2 = blend(a, val_2, filtered_value_2);
    filtered_value_3 = blend(a, val_3, filtered_value_3);
    filtered_value_4 = blend(a, val_4, filtered_value_4);

    float normalized_0 = flex_normalize_voltage(filtered_value_0);
    float normalized_1 = flex_normalize_voltage(filtered_value_1);
    float normalized_2 = flex_normalize_voltage(filtered_value_2);
    float normalized_3 = flex_normalize_voltage(filtered_value_3);
    float normalized_4 = flex_normalize_voltage(filtered_value_4);

    float angle1 = mpuGetInbetweenDeg(mpuBack, mpuIndex) * M_PI / 180.0f;
    float angle2 = mpuGetInbetweenDeg(mpuBack, mpuMiddle) * M_PI / 180.0f; 
    float angle3 = mpuGetInbetweenDeg(mpuBack, mpuRing) * M_PI / 180.0f;
    float angle4 = mpuGetInbetweenDeg(mpuBack, mpuPinky) * M_PI / 180.0f;

    // float angle1 = 60.209991 ;
    // float angle2 = 62.348457 ;
    // float angle3 = 60.422966 ;
    // float angle4 = 62.245049 ;


    if (esp_timer_get_time() - currTime > updateTime){
      ESP_LOGI("MAIN", "%f,%f,%f,%f,%f,%f,%f,%f,%f", normalized_0, normalized_1,
               normalized_2, normalized_3, normalized_4, angle1, angle2, angle3,
               angle4);

      std::array<float, 13> input{
          normalized_0,     normalized_1,     normalized_2,
          normalized_3,     normalized_4,     std::sin(angle1),
          std::cos(angle1), std::sin(angle2), std::cos(angle2),
          std::sin(angle3), std::cos(angle3), std::sin(angle4),
          std::cos(angle4)

      };

      // std::array<float_t, 13> input {
      //   -0.31149566173553467, -0.9407646656036377, -0.2282748967409134,
      //       0.001249765744432807, -0.013378581032156944, -1.4272600412368774,
      //       0.8681797981262207, -1.740282654762268, 1.068088173866272,
      //       0.04834785312414169, 0.22689466178417206, 0.2733452618122101,
      // 	     -0.04030461981892586
      // };

      for (int i = 0; i < input.size(); i++) {
	input.at(i) = (input[i] - SCALER_MEAN[i]) / SCALER_STD[i];
      }
      
      int out = tfInference(input.data());

      ESP_LOGI("MAIN", "out: %c", LABELS_NAME[out]);
      currTime = esp_timer_get_time();
    }

    // ESP_LOGI("MAIN", "angle: %f : %f : %f : %f", angle1, angle2, angle3,
    // angle4);
    // ESP_LOGI("MAIN", "pitch: %f : %f : %f : %f : %f", pitch1, pitch2, pitch3,
    // 	     pitch4, pitch5);

  }
}

// void btn_isr_handler(void *arg) {
//   xQueueSendFromISR(intr_queue, nullptr, nullptr);
// }

// void report_flex_values(void *args) {
//   while (1) {
//     if (xQueueReceive(intr_queue, nullptr, portMAX_DELAY)) {
//       isPause = !isPause;

//       ESP_LOGI("BTN", "isPause: %d", isPause);
//       // float prevVal = flex_read(ADC_CHANNEL_3);
//       // vTaskDelay(pdMS_TO_TICKS(1000)); // for smoothing
  
//       // float val = flex_read(ADC_CHANNEL_3);

//       // prevVal = blend(alpha, val, prevVal);
//       // float normalized = flex_normalize_voltage(prevVal);

//       // ESP_LOGI("BTN", "\t reading result (voltage):\t %f", prevVal);
//       // ESP_LOGI("BTN", "\t reading result (normalized):\t %f", normalized);
//     }
//   }
// }
  
