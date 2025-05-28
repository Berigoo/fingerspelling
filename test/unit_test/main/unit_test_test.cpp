#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "unity.h"
#include "mpu60x0.h"
#include "unity_test_runner.h"
#include "driver/i2c_master.h"
#include <cmath>
#include <cstdint>
#include "utils.h"


static float normalizePitch(float pitch) {
    if (pitch > 0) {
        return -180 + pitch;  // Converts 0-90 to -180 to -90
    }
    return pitch;  // Keeps -90 to 0 as is
}

extern "C"
void app_main(void)
{
  /* These are the different ways of running registered tests.
   * In practice, only one of them is usually needed.
   *
   * UNITY_BEGIN() and UNITY_END() calls tell Unity to print a summary
   * (number of tests executed/failed/ignored) of tests executed between these
   * calls.
   */
    // UNITY_BEGIN();
    // unity_run_test_by_name("PROBING");
    // unity_run_test_by_name("GYRO");
    // unity_run_test_by_name("ACC");
    // unity_run_test_by_name("LOW MODE");
    // unity_run_test_by_name("CYCLE MODE");
    // unity_run_test_by_name("SLEEP MODE");
    // unity_run_test_by_name("DISABLE GYRO AXIS");
    // unity_run_test_by_name("DISABLE ACC AXIS");
    // UNITY_END();
    
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.scl_io_num = GPIO_NUM_4; // TODO kconfig
    i2c_mst_config.sda_io_num = GPIO_NUM_5;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &handle));
    Mpu mpu(handle, 400000, false, 512.0f, 0.9);
    Mpu mpu2(handle, 400000, true, 512.0f, 0.9);

    bool isSampled = false;
    float bias = 0.0f;
    int i = 0;
    float *yawVal1 = new float[200];

    float prev = 0;
    float prev2 = 0;
    float prevX = 0;
    float prevX2 = 0;
    while (1) {
      mpu.update();
      mpu2.update();

      // if (!isSampled) {
      // 	float next = mpu.getYaw() * M_PI / 180.0f;
      //   yawVal1[i] = atan2(sin(next - prevVal), cos(next - prevVal)) * 180.0f / M_PI;
      // 	ESP_LOGI("CALC", "diff - %d: %f", i, yawVal1[i]);
      // 	prevVal = next;
      //   i++;

      //   if (i == 200 - 1) {
      //     float sum = 0.0f;
      //     for (int j = 0; j < 200; j++) {
      // 	    sum += yawVal1[j];
      //     }
      //     bias = sum / 200;
      // 	  ESP_LOGI("CALC", "bias: %f", bias);
      // 	  isSampled = true;
      //   }
      // 	vTaskDelay(50 / portTICK_PERIOD_MS);
      // }

      // if (!isSampled) {
      // 	const float  alpha = 0.5;
      //   float val = mpu.getYaw() * M_PI / 180.0f;

      //   float x =  atan2(sin(val), cos(val));
      //   float y = atan2(sin(val - prev), cos(val - prev));

      //   float res = alpha * (prevX) + (1.0f - alpha) * (x - y);
      // 	prev = x;
      //   prevX = res;

      // 	float val2 = mpu2.getYaw() * M_PI / 180.0f;

      //   float x2 =  atan2(sin(val2), cos(val2));
      //   float y2 = atan2(sin(val2 - prev2), cos(val2 - prev2));

      //   float res2 = alpha * (prevX2) + (1.0f - alpha) * (x2 - y2);
      // 	prev2 = x2;
      //   prevX2 = res;

      //   float r = res2 - res;
      // 	float r2 = val2 - val;
      // 	ESP_LOGI("TEST", "%f - %f = %f | without: %f | y = %f", res2, res, r, r2, y);
      // 	prev = val;
      // }

      // Inbetween Deg
      // ESP_LOGI("main", "(%f, %f) : (%f, %f) =  %f", mpu.getRoll(), mpu.getPitch(), mpu2.getRoll(), mpu2.getPitch(),  mpuGetInbetweenDeg(mpu, mpu2));
      
      // Euler check
      float r = mpu.getRoll();
      float p = mpu.getPitch();
      float y = mpu.getYaw();
      float r2 = mpu2.getRoll();
      float p2 = mpu2.getPitch();
      float y2 = mpu2.getYaw();


      ESP_LOGI("main", "%f, %f, %f", r, p, y);
    }
}














