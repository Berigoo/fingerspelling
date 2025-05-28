#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "flex_sensor.h"
#include "button.h"
#include "freertos/idf_additions.h"
#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "helper_3dmath.h"
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "training_data.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "soft_i2c_master.h"
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>
#include "tf.h"

#define CONTACT_PIN GPIO_NUM_38

static const float a = 0.5; // for basic linear interpolation

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

  // reserving gpio for input
  {
    gpio_config_t contact = {};
    contact.intr_type = GPIO_INTR_DISABLE;
    contact.mode = GPIO_MODE_INPUT;
    contact.pull_down_en = GPIO_PULLDOWN_DISABLE;
    contact.pull_up_en = GPIO_PULLUP_ENABLE;
    contact.pin_bit_mask = (1ULL << CONTACT_PIN);
    gpio_config(&contact);
  }

  // reserving i2c controllers (hardware & software)
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

  // reserving adc units
  s_adcUnitContext = adc_unit_create(ADC_UNIT_1);
  s_adcUnitContext2 = adc_unit_create(ADC_UNIT_2);

  // create mpu objects
  MPU6050 mpuBack(s_i2cHandle2, 400000, false);
  MPU6050 mpuIndex(s_i2cHandle1, 400000, false);
  MPU6050 mpuMiddle(s_i2cHandle1, 400000, true);
  MPU6050 mpuRing(s_i2cHandle2, 400000, true);
  MPU6050 mpuPinky(s_i2cHandle3, false);
  MPU6050 mpuThumb(s_i2cHandle3, true);

  // assigning adc units
  s_flexThumbContext = flex_create(s_adcUnitContext, ADC_CHANNEL_5); // GPIO 6
  s_flexIndexContext = flex_create(s_adcUnitContext2, ADC_CHANNEL_9); // GPIO 20
  s_flexMiddleContext = flex_create(s_adcUnitContext, ADC_CHANNEL_7); // GPIO 8
  s_flexRingContext = flex_create(s_adcUnitContext2, ADC_CHANNEL_6); // GPIO 17
  s_flexPinkyContext = flex_create(s_adcUnitContext2, ADC_CHANNEL_7); // GPIO 18

  // read adc
  float filtered_value_0 = flex_read(s_flexThumbContext);
  float filtered_value_1 = flex_read(s_flexIndexContext);
  float filtered_value_2 = flex_read(s_flexMiddleContext);
  float filtered_value_3 = flex_read(s_flexRingContext);
  float filtered_value_4 = flex_read(s_flexPinkyContext);

  mpuBack.initialize();
  mpuIndex.initialize();
  mpuMiddle.initialize();
  mpuRing.initialize();
  mpuPinky.initialize();
  mpuThumb.initialize();

  mpuBack.dmpInitialize();
  mpuIndex.dmpInitialize();
  mpuMiddle.dmpInitialize();
  mpuRing.dmpInitialize();
  mpuPinky.dmpInitialize();
  mpuThumb.dmpInitialize();

  mpuBack.setDMPEnabled(true);
  mpuIndex.setDMPEnabled(true);
  mpuMiddle.setDMPEnabled(true);
  mpuRing.setDMPEnabled(true);
  mpuPinky.setDMPEnabled(true);
  mpuThumb.setDMPEnabled(true);
  
  while (1) {
    // updating quaternion & clearing fifo
    mpuBack.update(42);
    mpuIndex.update(42);
    mpuMiddle.update(42);
    mpuRing.update(42);
    mpuPinky.update(42);
    mpuThumb.update(42);
    
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

    // v
    float normalized_0 = flex_normalize_voltage(filtered_value_0);
    float normalized_1 = flex_normalize_voltage(filtered_value_1);
    float normalized_2 = flex_normalize_voltage(filtered_value_2);
    float normalized_3 = flex_normalize_voltage(filtered_value_3);
    float normalized_4 = flex_normalize_voltage(filtered_value_4);

    // get relative quaternion to back mpu
    Quaternion qBack = mpuBack.getQuaternion();
    Quaternion qIndexRel =
        mpuIndex.getQuaternion().getConjugate().getProduct(qBack);
    Quaternion qMiddleRel =
        mpuMiddle.getQuaternion().getConjugate().getProduct(qBack);
    Quaternion qRingRel =
        mpuRing.getQuaternion().getConjugate().getProduct(qBack);
    Quaternion qPinkyRel =
        mpuPinky.getQuaternion().getConjugate().getProduct(qBack);
    Quaternion qThumbRel =
        mpuThumb.getQuaternion().getConjugate().getProduct(qBack);

    bool isContacted = !gpio_get_level(CONTACT_PIN);

    ESP_LOGI("MPU", "MPU: back: %f %f %f %f", mpuBack.getQuaternion().w,
             mpuBack.getQuaternion().x, mpuBack.getQuaternion().y,
             mpuBack.getQuaternion().z);
    ESP_LOGI("MPU", "MPU: index: %f %f %f %f", mpuIndex.getQuaternion().w,
             mpuIndex.getQuaternion().x, mpuIndex.getQuaternion().y,
             mpuBack.getQuaternion().z);
    ESP_LOGI("MPU", "MPU: middle: %f %f %f %f", mpuMiddle.getQuaternion().w,
             mpuMiddle.getQuaternion().x, mpuMiddle.getQuaternion().y,
             mpuMiddle.getQuaternion().z);
    ESP_LOGI("MPU", "MPU: ring: %f %f %f %f", mpuRing.getQuaternion().w,
             mpuRing.getQuaternion().x, mpuRing.getQuaternion().y,
             mpuRing.getQuaternion().z);
    ESP_LOGI("MPU", "MPU: pinky: %f %f %f %f", mpuPinky.getQuaternion().w,
             mpuPinky.getQuaternion().x, mpuPinky.getQuaternion().y,
             mpuPinky.getQuaternion().z);
    ESP_LOGI("MPU", "MPU: thumb: %f %f %f %f", mpuThumb.getQuaternion().w,
             mpuThumb.getQuaternion().x, mpuThumb.getQuaternion().y,
             mpuThumb.getQuaternion().z);
  }
}
