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
#include "portmacro.h"
#include "soc/gpio_num.h"
#include "training_data.h"
#include "utils.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "soft_i2c_master.h"
#include <array>
#define _USE_MATH_DEFINES
#include <math.h>
#include "tf.h"

#define CONTACT_PIN GPIO_NUM_38

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


uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
// TODO become a member class 
static uint16_t fifoCount0;
static uint8_t fifoBuffer0[64];
static float euler0[3];
static uint16_t fifoCount1;
static uint8_t fifoBuffer1[64];
static float euler1[3];
static uint16_t fifoCount2;
static uint8_t fifoBuffer2[64];
static float euler2[3];
static uint16_t fifoCount3;
static uint8_t fifoBuffer3[64];
static float euler3[3];
static uint16_t fifoCount4;     
static uint8_t fifoBuffer4[64];
static float euler4[3];
static uint16_t fifoCount5;     
static uint8_t fifoBuffer5[65];
static float euler5[3];
// TODO dispatch to a task
static bool getEulerMPU(MPU6050 &mpu, uint8_t *fifoBuffer, uint16_t &fifoCount,
                        uint16_t packetSize, float out[3]);


extern "C" void app_main(void) {
  tfSetup();

  {
    gpio_config_t contact = {};
    contact.intr_type = GPIO_INTR_DISABLE;
    contact.mode = GPIO_MODE_INPUT;
    contact.pull_down_en = GPIO_PULLDOWN_DISABLE;
    contact.pull_up_en = GPIO_PULLUP_ENABLE;
    contact.pin_bit_mask = (1ULL << CONTACT_PIN);
    gpio_config(&contact);
  }
  
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

  MPU6050 mpuBack(s_i2cHandle2, 400000, false);
  MPU6050 mpuIndex(s_i2cHandle1, 400000, false);
  MPU6050 mpuMiddle(s_i2cHandle1, 400000, true);
  MPU6050 mpuRing(s_i2cHandle2, 400000, true);
  MPU6050 mpuPinky(s_i2cHandle3, false);
  MPU6050 mpuThumb(s_i2cHandle3, true);


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

  // ----
  {
    gpio_config_t btn = {};
    btn.intr_type = GPIO_INTR_DISABLE;
    btn.mode = GPIO_MODE_INPUT;
    btn.pull_down_en = GPIO_PULLDOWN_ENABLE;
    btn.pull_up_en = GPIO_PULLUP_DISABLE;
    btn.pin_bit_mask = (1ULL << GPIO_NUM_39);
    gpio_config(&btn);
  }
  bool isPressed = false;
  bool isToggled = false;
  // ----
  while (1) {
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

    float normalized_0 = flex_normalize_voltage(filtered_value_0);
    float normalized_1 = flex_normalize_voltage(filtered_value_1);
    float normalized_2 = flex_normalize_voltage(filtered_value_2);
    float normalized_3 = flex_normalize_voltage(filtered_value_3);
    float normalized_4 = flex_normalize_voltage(filtered_value_4);

    // float angle1 = mpuGetInbetweenDeg(mpuBack, mpuIndex) * M_PI / 180.0f;
    // float angle2 = mpuGetInbetweenDeg(mpuBack, mpuMiddle) * M_PI / 180.0f;
    // float angle3 = mpuGetInbetweenDeg(mpuBack, mpuRing) * M_PI / 180.0f;
    // float angle4 = mpuGetInbetweenDeg(mpuBack, mpuPinky) * M_PI / 180.0f;
    // float angle5 = mpuGetInbetweenDeg(mpuBack, mpuThumb) * M_PI / 180.0f;
    // float angle1 = () * M_PI / 180.0f;
    // float angle2 = mpuGetInbetweenDeg(mpuBack, mpuMiddle) * M_PI / 180.0f; 
    // float angle3 = mpuGetInbetweenDeg(mpuBack, mpuRing) * M_PI / 180.0f;
    // float angle4 = mpuGetInbetweenDeg(mpuBack, mpuPinky) * M_PI / 180.0f;
    // float angle5 = mpuGetInbetweenDeg(mpuBack, mpuThumb) * M_PI / 180.0f; 
    // float angle1 = 60.209991 ;
    // float angle2 = 62.348457 ;
    // float angle3 = 60.422966 ;
    // float angle4 = 62.245049 ;

    // float backPitch = mpuBack.getPitch() * M_PI / 180.0f;
    // float backRoll = mpuBack.getRoll() * M_PI / 180.0f;
    // float backPitch = euler0[1] * M_PI / 180.0f;
    // float backRoll = euler0[2] * M_PI / 180.0f;
    
    bool isContacted = !gpio_get_level(CONTACT_PIN);

    // if (esp_timer_get_time() - currTime > updateTime) {
    //   float p1 = mpuBack.getPitch();
    //   float r1 = mpuBack.getRoll();
    //   float p2 = mpuThumb.getPitch();
    //   float r2 = mpuThumb.getRoll();
    //   float d = std::abs(r2) - std::abs(r1);
    //   float rel = ((p2 * std::cos(45.0f * M_PI/180)) - (p1 * std::cos(45.0f * M_PI / 180.0f))) * M_PI / 180.0f;

    //   std::array<float, 20> input{
    //       normalized_0,       normalized_1,       normalized_2,
    //       normalized_3,       normalized_4,      std::sin(angle1),
    //       std::cos(angle1),   std::sin(angle2),   std::cos(angle2),
    //       std::sin(angle3),   std::cos(angle3),   std::sin(angle4),
    //       std::cos(angle4),   std::sin(rel),      std::cos(rel),
    //       std::sin(backRoll), std::cos(backRoll), std::sin(backPitch),
    //       std::cos(backPitch), (isContacted) ? 1.0f : 0.0f};


    //   // std::array<float_t, 20> input{0.881502628326416,    1.2652605772018433,
    //   //                               1.1429119110107422,   0.4233283996582031,
    //   //                               0.8364360332489014,   1.8322868347167969,
    //   //                               -1.2049839496612549,  1.0300217866897583,
    //   //                               0.19515833258628845,  0.999482274055481,
    //   //                               -0.30864647030830383, 0.9977184534072876,
    //   //                               -0.28897514939308167, 1.0515714883804321,
    //   //                               0.472451776266098,    0.19714051485061646,
    //   //                               0.31169363856315613,  -0.8495103716850281,
    //   //                               -1.2806310653686523,  1.0};
      

    //   for (int i = 0; i < NUM_FEATURES; i++) {
    // 	input.at(i) = (input[i] - SCALER_MEAN[i]) / SCALER_STD[i];
    //   }

    //   int out = tfInference(input);

    //   ESP_LOGI("MAIN", "out: %c", LABELS_NAME[out]);
    //   currTime = esp_timer_get_time();
    // }

    

    // if (gpio_get_level(GPIO_NUM_39) && isToggled) {
    //   // ESP_LOGI("MAIN", "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d", normalized_0,
    //   //          normalized_1, normalized_2, normalized_3, normalized_4,
    //   //          backPitch, backRoll, angle1, angle2, angle3, angle4, angle5,
    //   //          isContacted);
    //   printf( "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n", normalized_0,
    //            normalized_1, normalized_2, normalized_3, normalized_4,
    //            backPitch, backRoll, angle1, angle2, angle3, angle4, rel, isContacted);
    //   isToggled = false;
    // } else if (!gpio_get_level(GPIO_NUM_39) && !isToggled) {
    //   isToggled = true;
    // }
     // printf( "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d\n", normalized_0,
     //           normalized_1, normalized_2, normalized_3, normalized_4,
     //           backPitch, backRoll, angle1, angle2, angle3, angle4, rel, isContacted);
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

bool getEulerMPU(MPU6050 &mpu, uint8_t *fifoBuffer, uint16_t &fifoCount,
                 uint16_t packetSize, float out[3]) {
  uint8_t mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    return false;
    // otherwise, check for DMP data ready interrupt frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    Quaternion q;
    VectorFloat gravity;
    
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(out, &q, &gravity);

    // printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
    // printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
    // printf("ROLL: %3.1f \n", ypr[2] * 180 / M_PI);
    return true;
  }
  return false;
}
