#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "portmacro.h"
#include "unity.h"
#include "mpu60x0.h"
#include "unity_test_runner.h"
#include "driver/i2c_master.h"
#include <cmath>
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
    // unity_run_test_by_name("Init");
    // unity_run_test_by_name("GYRO");
    // unity_run_test_by_name("ACC");
    // unity_run_test_by_name("LOW MODE");
    // unity_run_test_by_name("SLEEP MODE");
    // unity_run_test_by_name("DISABLE GYRO AXIS");
    // unity_run_test_by_name("DISABLE ACC AXIS");
    // UNITY_END();
    
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.scl_io_num = GPIO_NUM_9; // TODO kconfig
    i2c_mst_config.sda_io_num = GPIO_NUM_10;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &handle));
    Mpu mpu(handle, 400000, false, 512.0f, 0.9);
    Mpu mpu2(handle, 400000, true, 512.0f, 0.9);
    while (1) {
      mpu.update();
      mpu2.update();

      // Inbetween Deg
      // ESP_LOGI("main", "(%f, %f) : (%f, %f) =  %f", mpu.getRoll(), mpu.getPitch(), mpu2.getRoll(), mpu2.getPitch(),  mpuGetInbetweenDeg(mpu, mpu2));
      
      // Euler check
      float r = mpu.getRoll();
      float p = mpu.getPitch();
      float y = mpu.getYaw();
      float rp = mpu.getPitchRadians();

      float r2 = mpu2.getRoll();
      float p2 = mpu2.getPitch();
      float y2 = mpu2.getYaw();
      float rp2 = mpu2.getPitchRadians();

      // mpuGetInbetweenDeg(mpu, mpu2);
      
      ESP_LOGI("main", "out: (%f) (%f) = %f", p * 180 / M_PI, p2 * 180 / M_PI, mpuGetInbetweenDeg(mpu, mpu2) * 180 / M_PI);
    }
}
