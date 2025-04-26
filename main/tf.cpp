#include "tf.h"
#include "esp_log.h"
#include "model.h"
#include "training_data.h"
static const tflite::Model* model = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor* input = nullptr;
static TfLiteTensor* output = nullptr;
static int inference_count = 0;

static constexpr int kTensorArenaSize = 100 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

static int argmax(float* in);

void tfSetup() {
  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(model_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf("Model provided is schema version %d not equal to supported "
                "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  static tflite::MicroMutableOpResolver<3> resolver;
  if (resolver.AddFullyConnected() != kTfLiteOk) {
    return;
  }
  if (resolver.AddRelu() != kTfLiteOk) {
    return;
  }
  if (resolver.AddSoftmax() != kTfLiteOk) {
    return;
  }
  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
						     model, resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  // Keep track of how many inferences we have performed.
  inference_count = 0;
}

int tfInference(float *inputs) {
  for (int i=0; i<NUM_FEATURES; i++) {
    input->data.f[i] = inputs[i];
  }
  
  ESP_LOGI("TF", "scale: %f", input->params.scale);

  printf("\t------------------\n");
  for (int i = 0; i < 13; i++) {
    printf("%f, ", input->data.f[i]);
  }
  printf("\n\t------------------\n");
  
   // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    MicroPrintf("Invoke failed\n");
    return -100;
  }

  return argmax(output->data.f);
}

int argmax(float* arr){
  float temp = 0.0f;
  int index = 0;
  for(size_t i=0; i<NUM_CLASSES; i++){
    float curr = arr[i];
    if(curr > temp){
      temp = curr;
      index = i;
    }
  }
  ESP_LOGI("TF", "pred: %f, index: %d", temp, index);
  return index;
}
