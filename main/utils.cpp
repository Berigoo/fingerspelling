#include "utils.h"

float normalization(float val, float min, float max) {
  return (val - min) / (max - min);
}

float blend(float alpha, float new_val, float prev_val) {
  return (alpha * new_val) + ((1 - alpha) * prev_val);
}
