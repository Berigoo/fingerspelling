#ifndef MAIN_UTILS
#define MAIN_UTILS

constexpr float search_vt(float vs, float r1, float r2) {
  return vs * (r2 / (r1 + r2));
}

float normalization(float val, float min, float max);  //TODO maybe implement RelU

float blend(float alpha, float new_val, float prev_val);

#endif // MAIN_UTILS
