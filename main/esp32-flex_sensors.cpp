#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hal/adc_types.h"

#define TAG "ADC"

#define ADC_SEL_ATTEN		ADC_ATTEN_DB_12
#define ADC_SEL_UNIT		ADC_UNIT_1
#define ADC_SEL_ULP_MODE	ADC_ULP_MODE_DISABLE
#define ADC_SEL_CHAN_0		ADC_CHANNEL_3

constexpr float search_vt(float vs, float r1, float r2) {
  return vs * (r2 / (r1 + r2));
}
static float normalization(float val, float min, float max);  //TODO maybe implement RelU

static void adc_init(adc_oneshot_unit_handle_t &handle);
static void adc_config(adc_oneshot_unit_handle_t &handle);
static bool adc_config_calibrator(adc_cali_handle_t &cali_handle);
static void adc_clear_calibrator(adc_cali_handle_t &cali_handle);
static void adc_clear(adc_oneshot_unit_handle_t &handle);

//TODO
// the range is about 1V from straight to fully bent
// max is about 0.95
// min is about -0.02
#define V_REF 3.3f
#define R1 24000
#define R2_FLEX_MIN 12000
#define R2_FLEX_MAX 40000

constexpr float VOLTAGE_MIN = search_vt(V_REF, R1, R2_FLEX_MIN);
constexpr float VOLTAGE_MAX = search_vt(V_REF, R1, R2_FLEX_MAX);

extern "C"
void app_main(void)
{

  ESP_LOGI(TAG, "min: %f ; max: %f", VOLTAGE_MIN, VOLTAGE_MAX);
  adc_oneshot_unit_handle_t adc_handle;
  adc_cali_handle_t adc_cali_handle;
  adc_init(adc_handle);
  adc_config(adc_handle);
  adc_config_calibrator(adc_cali_handle);

  while(1) {
    int raw, mvoltage;
    float voltage;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_SEL_CHAN_0, &raw));
    ESP_LOGI(TAG, "ADC%d Channel[%d] raw: %d", ADC_SEL_UNIT+1, ADC_SEL_CHAN_0, raw);
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &mvoltage));
    voltage = mvoltage / 1000.0f;
    ESP_LOGI(TAG, "ADC%d Channel[%d] V: %f", ADC_SEL_UNIT+1, ADC_SEL_CHAN_0, voltage);
    ESP_LOGI(TAG, "ADC%d Channel[%d] normalized: %f", ADC_SEL_UNIT+1, ADC_SEL_CHAN_0,
	     (normalization(voltage, VOLTAGE_MIN, VOLTAGE_MAX)));
    
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  
  adc_clear_calibrator(adc_cali_handle);
  adc_clear(adc_handle);
}

void adc_init(adc_oneshot_unit_handle_t &handle) {
  adc_oneshot_unit_init_cfg_t config = {
    .unit_id = ADC_SEL_UNIT,
    .ulp_mode = ADC_SEL_ULP_MODE
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&config, &handle));
}

void adc_config(adc_oneshot_unit_handle_t &handle) {
  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_SEL_ATTEN,
    .bitwidth = ADC_BITWIDTH_DEFAULT
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_SEL_CHAN_0, &config));
}

bool adc_config_calibrator(adc_cali_handle_t &cali_handle) {
  bool calibrated = false;
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!calibrated) {
    ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_SEL_UNIT,
      .chan = ADC_SEL_CHAN_0,
      .atten = ADC_SEL_ATTEN,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    if (ret == ESP_OK) {
      calibrated = true;
    }
  }
#endif
  return calibrated;
}

void adc_clear_calibrator(adc_cali_handle_t &cali_handle) {
  adc_clear_calibrator(cali_handle);
}

		  
void adc_clear(adc_oneshot_unit_handle_t &handle) {
  ESP_ERROR_CHECK(adc_oneshot_del_unit(handle));
}


float normalization(float val, float min, float max) {
  return (val - min) / (max - min);
}
