#include "utils.h"
#include "esp_log.h"
#include <cmath>
#include <algorithm>
#include <cmath>
#include <limits>

float normalization(float val, float min, float max) {
  return (val - min) / (max - min);
}

float blend(float alpha, float new_val, float prev_val) {
  return (alpha * new_val) + ((1 - alpha) * prev_val);
}

float mpuGetInbetweenDeg(Mpu &head, Mpu &tail, uint8_t &&target) {
  float theta1, y;
  int nose1=1, nose2=1;
  switch (target) {
  case 1:
    theta1 = head.getPitch();
    y = tail.getPitch();
    nose1 = (abs(head.getRoll()) > 95) ? -1 : 1;
    nose2 = (abs(head.getRoll()) > 95) ? -1 : 1;
    break;
  case 2:
    theta1 = head.getYaw();
    y = tail.getYaw();
    break;
  default:
    theta1 = head.getRoll();
    y = tail.getRoll();
    break;
  }

  float out = (-nose1 * theta1) + (nose2 * y);

  if (out < 0 && nose1 < 0)
    out += 180;
  else if (out > 0 && nose1 < 0)
    out += 180;
  else if (out < 0 && nose1 > 0)
    out += 360;
  return out;
}

// int inference(std::array<float, 13> &input) {
//   std::array<float, 13> tmp = input;
//   // preprocess input
//   for (int i = 5; i < 9; i++) {
//     tmp[i] = std::sin(input[i] * M_PI / 180);
//     tmp[i+1] = std::cos(input[i] * M_PI / 180);
//   }

//   ESP_LOGI("INF", "angle1: %f %f", tmp[5], tmp[6]);
//   for (int i = 0; i < NUM_FEATURES; i++) {
//     tmp[i] = (tmp[i] - SCALER_MEAN[i]) / SCALER_STD[i];
//   }

//   // distance calc
//   std::array<float, NUM_SAMPLES> dists;
//   for (int i = 0; i < NUM_SAMPLES; i++) {
//     float dist = 0;
//     for (int j = 0; j < NUM_FEATURES; j++) {
//       dist += std::pow(tmp[j] - TRAINING_DATA[i][j], 2);
//     }
//     dists[i] = dist;
//   }

//   ESP_LOGI("INF", "dist: %f %f", dists[9], dists[11]);
//   // voting
// #define K 3
//   std::array<int, K> selIndices;
//   for (auto &index : selIndices) {
//     auto a = std::min_element(dists.begin(), dists.end());
//     auto i = std::distance(dists.begin(), a);
//     ESP_LOGI("INF", "min val: %f", *a);
//     dists.at(i) = std::numeric_limits<float>::max();
//     index = i;
//   }

//   int selIndex = selIndices[0];
//   if (selIndices[1] == selIndices[2])
//     selIndex = selIndices[1];

//   ESP_LOGI("INF", "%d %d %d", selIndices[0], selIndices[1], selIndices[2]);
//   return selIndex;
// }
