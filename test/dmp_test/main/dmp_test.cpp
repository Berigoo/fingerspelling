#include "driver/i2c_types.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "helper_3dmath.h"
#include "portmacro.h"
#include "driver/i2c_master.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "soft_i2c_master.h"
#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>
#include "MadgwickAHRS.h"
#include <cstdint>
#include "esp_timer.h"

static size_t m_lastUpdateTime;
static size_t m_filterTargetDelay = 1000000 / 512;

static VectorFloat gyroOff = {};
static VectorFloat accelOff = {};

static Quaternion mpuQInitial;
static Quaternion mpuQInitial1;

static void calibrate(MPU6050& mpu) {
  // float sumGyroX = 0.0f, sumGyroY = 0.0f, sumGyroZ = 0.0f;
  // float sumAccX = 0.0f, sumAccY = 0.0f, sumAccZ = 0.0f;
  // const int samples = 200;

  // for (int i = 0; i < samples; i++) {
  //   VectorInt16 gyro;
  //   VectorInt16 accel;

  //   mpu.getMotion6(&accel.x, &accel.y, &accel.z, &gyro.x, &gyro.y, &gyro.z);
      
  //   sumGyroX += gyro.x;
  //   sumGyroY += gyro.y;
  //   sumGyroZ += gyro.z;
    
  //   sumAccX += accel.x;
  //   sumAccY += accel.y;
  //   sumAccZ += accel.z;

  //   vTaskDelay(1 / portTICK_PERIOD_MS);
  // }

  // m_offsetGyro.x = sumGyroX / samples;
  // m_offsetGyro.y = sumGyroY / samples;
  // m_offsetGyro.z = sumGyroZ / samples;

  // m_offsetAcc.x = sumAccX / samples;
  // m_offsetAcc.y = sumAccY / samples;
  // m_offsetAcc.z = sumAccZ / samples;
  
  // ESP_LOGI("mpu", "calibrated");	         
}

static void update(MPU6050 &mpu, Madgwick &madgwick, uint8_t* fifoBuffer, uint16_t& fifoCount, uint16_t packetSize, VectorFloat *out) {
  // uint8_t mpuIntStatus = mpu.getIntStatus();
  // // get current FIFO count
  // fifoCount = mpu.getFIFOCount();

  // if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
  //   // reset so we can continue cleanly
  //   mpu.resetFIFO();

  //   // otherwise, check for DMP data ready interrupt frequently)
  // } else if (mpuIntStatus & 0x02) {
  //   // wait for correct available data length, should be a VERY short wait
  //   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  //   // read a packet from FIFO
  //   VectorInt16 accelDmp;
  //   VectorInt16 gyroDmp;

  //   mpu.getFIFOBytes(fifoBuffer, packetSize);
  //   mpu.dmpGetAccel(&accelDmp, fifoBuffer);
  //   mpu.dmpGetGyro(&gyroDmp, fifoBuffer);

  //   // madgwick.updateIMU(gyroDmp.x, gyroDmp.y, gyroDmp.z, accelDmp.x, accelDmp.y, accelDmp.z);

  //   out->x = madgwick.getRoll();
  //   out->y = madgwick.getPitch();
  //   out->z = madgwick.getYaw();

  //   printf("Out angles x: %f, ", out->x);
  //   printf("y: %f, ", out->y);
  //   printf("z: %f ", out->z);

  // }
}
static bool getEuler(MPU6050 &mpu, uint8_t* fifoBuffer, uint16_t& fifoCount, uint16_t packetSize, Quaternion& intialFrame, Quaternion* out) {
  uint8_t mpuIntStatus = mpu.getIntStatus();
  // // get current FIFO count
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
    int16_t g[3];
    VectorInt16 accelDmp;
    VectorInt16 gyroDmp;

    VectorInt16 accelReal;
    VectorInt16 accelWorld;

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    q.normalize();
    mpu.dmpGetAccel(&accelDmp, fifoBuffer);
    accelDmp.x = accelDmp.x + accelOff.x;
    accelDmp.y = accelDmp.y + accelOff.y;
    accelDmp.z = accelDmp.z + accelOff.z;
    mpu.dmpGetGyro(&gyroDmp, fifoBuffer);
    gyroDmp.x = gyroDmp.x + gyroOff.x;
    gyroDmp.y = gyroDmp.y + gyroOff.y;
    gyroDmp.z = gyroDmp.z + gyroOff.z;
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetGravity(g, fifoBuffer);

    VectorFloat gravityNED;
    gravityNED.x = 0.0;
    gravityNED.y = 0.0;
    gravityNED.z = 1.0f;
    mpu.dmpGetLinearAccel(&accelReal, &accelDmp, &gravity); // 
    mpu.dmpGetLinearAccelInWorld(&accelWorld, &accelReal, &q); //

    //-----
    VectorFloat rawAccelScaled;
    VectorFloat accelScaledBody; // BODY
    VectorFloat accelScaled;     // WORLD
    // accelScaled.x = accelWorld.x / 16384.0f * 9.8f;
    // accelScaled.y = accelWorld.y / 16384.0f * 9.8f;
    // accelScaled.z = accelWorld.z / 16384.0f * 9.8f;
    rawAccelScaled.x = accelDmp.x / 16384.0f ; // 9.8f;
    rawAccelScaled.y = accelDmp.y / 16384.0f ; // 9.8f;
    rawAccelScaled.z = accelDmp.z / 16384.0f ; // 9.8f;
    accelScaled.x = accelDmp.x / 16384.0f * 9.8f;
    accelScaled.y = accelDmp.y / 16384.0f * 9.8f;
    accelScaled.z = accelDmp.z / 16384.0f * 9.8f;
    accelScaledBody.x = accelReal.x / 16384.0f * 9.8f;
    accelScaledBody.y = accelReal.y / 16384.0f * 9.8f;
    accelScaledBody.z = accelReal.z / 16384.0f * 9.8f;
    //-----
    accelScaled.rotate(&q);
    
    ESP_LOGI("MPU", "body: %f %f %f %f", q.w, q.x, q.y, q.z);
    ESP_LOGI("MPU", "world linaccel: %f %f %f", accelScaled.x, accelScaled.y,
             accelScaled.z);
    ESP_LOGI("MPU", "body linaccel: %f %f %f", accelScaledBody.x,
             accelScaledBody.y, accelScaledBody.z);
    ESP_LOGI("MPU", "body gravity: %d %d %d", g[0], g[1], g[2]);
    ESP_LOGI("MPU", "raw accel: %f %f %f", rawAccelScaled.x,
             rawAccelScaled.y, rawAccelScaled.z);
    
    Quaternion world_frame = intialFrame.getProduct(q);
    VectorFloat world_gravity;

    // mpu.dmpGetGravity(&world_gravity, &world_frame);
    // mpu.dmpGetYawPitchRoll(out, &world_frame, &world_gravity);

    // gyroFloat.x = 1.0f * (gyro.x) / 131.0f;
    // gyroFloat.y = 1.0f * (gyro.y) / 131.0f;
    // gyroFloat.z = 1.0f * (gyro.z) / 131.0f;
    // accelFloat.x = 1.0f * (accel.x) / 16384.0f;
    // accelFloat.y = 1.0f * (accel.y) / 16384.0f;
    // accelFloat.z = 1.0f * (accel.z) / 16384.0f;

    uint64_t now_us = esp_timer_get_time();
    uint64_t dt = now_us - m_lastUpdateTime;

    if (dt < m_filterTargetDelay)
      return false;
    
    // madgwick.updateIMU(gyroFloat.x, gyroFloat.y, gyroFloat.z, accelFloat.x, accelFloat.y, accelFloat.z);

    // printf("accel x: %f, ", out[2] * 180.0f / M_PI);
    // printf("y: %f, ", out[1] * 180.0f / M_PI);
    // printf("z: %f\n", out[0] * 180.0f / M_PI);
    // printf("accel x: %f, ", madgwick.getRoll());
    // printf("y: %f, ", madgwick.getPitch());
    // printf("z: %f\n", madgwick.getYaw());

    // printf("x: %3.1f, ", q.x);
    // printf("y: %3.1f, ", q.y);
    // printf("z: %3.1f, ", q.z);
    // printf("w: %3.1f \n", q.w);    
    
    // printf("psi: %3.1f, ", out1[0]);
    // printf("theta: %3.1f, ", out1[1]);
    // printf("phi: %3.1f \n", out1[2]);

    m_lastUpdateTime = now_us;
    *out = world_frame;
    return true;
  }
  return {};
}

uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// TODO care about angle wrapping before averaging maybe use atan2
Quaternion getIntialQ(MPU6050 &mpu){
  const int samples = 500;
  const int threshold = 5000;
  int t_count = 0;
  int count = 0;
  Madgwick filter(512.0f, 0.8f);
  VectorFloat sum = {};
  while (count != samples) {
    VectorInt16 gyro;
    VectorInt16 accel;
    VectorFloat gyroF;
    VectorFloat accelF;

    mpu.getMotion6(&accel.x, &accel.y, &accel.z, &gyro.x, &gyro.y, &gyro.z);

    gyroF.x = 1.0f * gyro.x / 131.0f;
    gyroF.y = 1.0f * gyro.y / 131.0f;
    gyroF.z = 1.0f * gyro.z / 131.0f;

    accelF.x = 1.0f * accel.x / 16384.0f;
    accelF.y = 1.0f * accel.y / 16384.0f;
    accelF.z = 1.0f * accel.z / 16384.0f;

    filter.updateIMU(gyroF.x, gyroF.y, gyroF.z, accelF.x, accelF.y, accelF.z);

    if (t_count > threshold){
      sum.x += filter.getRollRadians();
      sum.y += filter.getPitchRadians();
      sum.z += filter.getYawRadians();
      
      count++;
    }
    t_count++;
    vTaskDelay(1000 / 512 / portTICK_PERIOD_MS);
  }

  sum.x = sum.x / samples;
  sum.y = sum.y / samples;
  sum.z = sum.z / samples;

  Quaternion q;
  q.w = filter.getQuaternion0();
  q.x = filter.getQuaternion1();
  q.y = filter.getQuaternion2();
  q.z = filter.getQuaternion3();
  q.normalize();

  return q;
}

Quaternion convertFromEuler(float radYaw, float radPitch, float radRoll) {
  Quaternion q;
  q.x = std::sin(radRoll/2.0f) * std::cos(radPitch/2.0f) * std::cos(radYaw/2.0f) - std::cos(radRoll/2.0f) * std::sin(radPitch/2.0f) * std::sin(radYaw/2.0f);
  q.y = std::cos(radRoll/2.0f) * std::sin(radPitch/2.0f) * std::cos(radYaw/2.0f) + std::sin(radRoll/2.0f) * std::cos(radPitch/2.0f) * std::sin(radYaw/2.0f);
  q.z = std::cos(radRoll/2.0f) * std::cos(radPitch/2.0f) * std::sin(radYaw/2.0f) - std::sin(radRoll/2.0f) * std::sin(radPitch/2.0f) * std::cos(radYaw/2.0f);
  q.w = std::cos(radRoll / 2.0f) * std::cos(radPitch / 2.0f) * std::cos(radYaw / 2.0f) +
    std::sin(radRoll / 2.0f) * std::sin(radPitch / 2.0f) * std::sin(radYaw / 2.0f);

  return q;
}

static Quaternion Qdiff(MPU6050 &mpu2, MPU6050 &mpu1, Quaternion &q2,
                        Quaternion &q1) {
  Quaternion Qres = q2.getConjugate();
  Qres = Qres.getProduct(q1);

  VectorFloat gravity;
  float ypr[3];
  mpu2.dmpGetGravity(&gravity, &q2);
  mpu2.dmpGetYawPitchRoll(ypr, &q2, &gravity);
  VectorFloat gravity1;
  float ypr1[3];
  mpu2.dmpGetGravity(&gravity1, &q1);
  mpu2.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);

  VectorFloat g;
  float ypr3[3];
  mpu2.dmpGetGravity(&g, &Qres);
  mpu2.dmpGetYawPitchRoll(ypr3, &Qres, &g);
  // printf("accel x: %f, ", ypr[2] * 180.0f / M_PI);
  // printf("y: %f, ", ypr[1] * 180.0f / M_PI);
  // printf("Yaw2: %f, Yaw1: %f = %f | %f\n", ypr1[2] * 180.0f / M_PI, ypr[2] * 180.0f / M_PI, (ypr[0] - ypr1[0]) * 180.0f / M_PI, ypr3[0] * 180.0f / M_PI);

  return Qres;
}

extern "C"
void app_main(void)
{
  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.i2c_port = I2C_NUM_0;
  i2c_mst_config.scl_io_num = GPIO_NUM_4; // TODO kconfig
  i2c_mst_config.sda_io_num = GPIO_NUM_5;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.flags.enable_internal_pullup = true;

  i2c_master_bus_handle_t handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &handle));

  soft_i2c_master_bus_t s_i2cHandle3;
  {
    soft_i2c_master_config_t conf{};
    conf.scl_pin = GPIO_NUM_12;
    conf.sda_pin = GPIO_NUM_13;
    conf.freq = SOFT_I2C_200KHZ;

    ESP_ERROR_CHECK(soft_i2c_master_new(&conf, &s_i2cHandle3));
  }

  MPU6050 mpu(handle, 400000, false);
  // MPU6050 mpu2(handle, 400000, true);

  mpu.initialize();
  mpu.dmpInitialize();
  // mpu2.initialize();
  // mpu2.dmpInitialize();

  vTaskDelay(100 / portTICK_PERIOD_MS);
  mpuQInitial = getIntialQ(mpu);
  // mpuQInitial = convertFromEuler(M_PI, mpuIntialEuler.y, mpuIntialEuler.x);
  // mpuQInitial.normalize();
  ESP_LOGI("MPU", "init: %f %f %f %f", mpuQInitial.w, mpuQInitial.x, mpuQInitial.y, mpuQInitial.z);
  // VectorFloat mpuIntialEuler1 = getIntialQ(mpu2);
  // mpuQInitial1 = convertFromEuler(M_PI, mpuIntialEuler.y, mpuIntialEuler.x);

    // printf("initial x: %f, y: %g, z: %f, w: %f, mag: %f\n", mpuIntialEuler.x, mpuIntialEuler.y, mpuIntialEuler.z, mpuQInitial.w, mpuQInitial.getMagnitude());
  
  // mpu.CalibrateAccel(15);
  // mpu.CalibrateGyro(15);
  // mpu2.CalibrateAccel(6);
  // mpu2.CalibrateGyro(6);  

  // gyroOff.x = mpu.getXGyroOffset();
  // gyroOff.y = mpu.getYGyroOffset();
  // gyroOff.z = mpu.getZGyroOffset();

  // accelOff.x = mpu.getXAccelOffset();
  // accelOff.y = mpu.getYAccelOffset();
  // accelOff.z = mpu.getZAccelOffset();

  // mpu.setXAccelOffset(accelOff.x);
  // mpu.setYAccelOffset(accelOff.y);
  // mpu.setZAccelOffset(accelOff.z);
  // mpu.setXGyroOffset(accelOff.x);
  // mpu.setYGyroOffset(accelOff.y);
  // mpu.setZGyroOffset(accelOff.z);
  
  mpu.setDMPEnabled(true);
  // mpu2.setDMPEnabled(true);

  // mpu2.initialize();
  // mpu2.dmpInitialize();
  // mpu2.CalibrateAccel(6);
  // mpu2.CalibrateGyro(6);
  // mpu2.setDMPEnabled(true);

  float out[3] = {0.0f, 0.0f, 0.0f};

  Quaternion m1;
  Quaternion m2;
  while (1) {
    if (getEuler(mpu, fifoBuffer, fifoCount, packetSize, mpuQInitial, &m1)) {
      // while(!getEuler(mpu2, fifoBuffer, fifoCount, packetSize, mpuQInitial1,
      // 		      &m2));
      // Qdiff(mpu2, mpu, m2, m1);
    } 
    // update(mpu, madgwick, fifoBuffer, fifoCount, packetSize, &euler);
    
    // VectorInt16 accel;
    // VectorInt16 gyro;
    // mpu.getAcceleration(&accel.x, &accel.y, &accel.z);
    // mpu.getRotation(&gyro.x, &gyro.y, &gyro.z);

    // getEuler(mpu2, fifoBuffer, fifoCount, packetSize, out2);
    // // ESP_LOGI("MAIN", "Y: %f, P: %f, R: %f", out[0] * 180 / M_PI, out[1] * 180.0f / M_PI, out[2] * 180.0f / M_PI);
    // ESP_LOGI("MAIN", "%f - %f = %f", out2[0] * 180.0f / M_PI, out[0] * 180.0f / M_PI, (float)(out2[0] - out[0]) * 180.0f / M_PI);
  }
}














