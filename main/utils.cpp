#include "utils.h"

float normalization(float val, float min, float max) {
  return (val - min) / (max - min);
}
