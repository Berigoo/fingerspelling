#ifndef MAIN_UTILS
#define MAIN_UTILS

#include <array>
#include "mpu60x0.h"

constexpr float search_vt(float vs, float r1, float r2) {
  return vs * (r2 / (r1 + r2));
}

float normalization(float val, float min, float max);  //TODO maybe implement RelU

float blend(float alpha, float new_val, float prev_val); // first order IIR

float mpuGetInbetweenDeg(Mpu &head, Mpu &tail, uint8_t &&target = 1);

/* int inference(std::array<float, 13>& input); /\* TODO *\/ */

#endif // MAIN_UTILS
