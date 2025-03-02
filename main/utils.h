#ifndef MAIN_UTILS
#define MAIN_UTILS

constexpr float search_vt(float vs, float r1, float r2) {
  return vs * (r2 / (r1 + r2));
}
float normalization(float val, float min, float max);  //TODO maybe implement RelU

#endif // MAIN_UTILS
