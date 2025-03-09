#include "flex_sensor.h"
#include "esp_adc/adc_oneshot.h"

//TODO
// the range is about 1V from straight to fully bent
// max is about 0.95
// min is about -0.02
#define V_REF		3.3f
#define R1		24000
#define R2_FLEX_MIN	12000
#define R2_FLEX_MAX	40000

constexpr float VOLTAGE_MIN = search_vt(V_REF, R1, R2_FLEX_MIN);
constexpr float VOLTAGE_MAX = search_vt(V_REF, R1, R2_FLEX_MAX);

struct AdcHandles {
  adc_oneshot_unit_handle_t unitHandle;
  adc_cali_handle_t caliHandle;
  bool calibrated = false;
};

static std::array<AdcHandles, FLEX_SENSOR_COUNT> s_adc_handles_lookup;

static void init_adc_channel(adc_unit_t unit, adc_channel_t channel, AdcHandles& handles);

void flex_init(int channel) {
  int count = 0;
  if (channel < 0) {
    for (auto &e : s_adc_handles_lookup) {
      init_adc_channel(ADC_UNIT, (adc_channel_t)(count+1), e);
      count++;
    }
  } else {
    init_adc_channel(ADC_UNIT, (adc_channel_t)channel, s_adc_handles_lookup[channel]);
  }
}

void flex_clear(int channel) {
  if (channel < 0) {
    for (auto &e : s_adc_handles_lookup) {
      ESP_ERROR_CHECK(adc_oneshot_del_unit(e.unitHandle));
      if (e.calibrated) {
        adc_cali_delete_scheme_curve_fitting(e.caliHandle);
      }
    }
  } else {
    ESP_ERROR_CHECK(adc_oneshot_del_unit(s_adc_handles_lookup[channel].unitHandle));
    if (s_adc_handles_lookup[channel].calibrated) {
      adc_cali_delete_scheme_curve_fitting(s_adc_handles_lookup[channel].caliHandle);
    }
  }
}

float flex_read(adc_channel_t channel) {
  int raw, mvoltage;
  float voltage = -1;
  AdcHandles* handles = &s_adc_handles_lookup[channel];
  ESP_ERROR_CHECK(adc_oneshot_read(handles->unitHandle, channel, &raw));
  ESP_LOGD("ADC", "ADC%d Channel[%d] raw: %d", ADC_UNIT + 1, channel, raw);
  if (handles->calibrated) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(handles->caliHandle, raw, &mvoltage));
    voltage = mvoltage / 1000.0f;
    ESP_LOGD("ADC", "ADC%d Channel[%d] V: %f", ADC_UNIT+1, channel, voltage);
    ESP_LOGD("ADC", "ADC%d Channel[%d] normalized: %f", ADC_UNIT + 1,
	     channel, (normalization(voltage, VOLTAGE_MIN, VOLTAGE_MAX)));
  }
  
  return voltage;
}

float flex_read_normalized(adc_channel_t channel) {
  AdcHandles *handles = &s_adc_handles_lookup[channel];
  if (handles->calibrated) {
    int raw, mvoltage;
    float voltage = -1, normalized;
  
    ESP_ERROR_CHECK(adc_oneshot_read(handles->unitHandle, channel, &raw));
    ESP_LOGD("ADC", "ADC%d Channel[%d] raw: %d", ADC_UNIT + 1, channel, raw);
  
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(handles->caliHandle, raw, &mvoltage));
    voltage = mvoltage / 1000.0f;
    normalized = normalization(voltage, VOLTAGE_MIN, VOLTAGE_MAX);
    
    ESP_LOGD("ADC", "ADC%d Channel[%d] V: %f", ADC_UNIT+1, channel, voltage);
    ESP_LOGD("ADC", "ADC%d Channel[%d] normalized: %f", ADC_UNIT + 1, channel,
	     normalized);

    return normalized;
  } else {
    ESP_LOGE("ADC", "ADC%d Channel[%d] not successfully calibrated", ADC_UNIT+1, channel);
    return -1.0f;
  }
}

float flex_normalize_voltage(float v) {
  return normalization(v, VOLTAGE_MIN, VOLTAGE_MAX);
}


void init_adc_channel(adc_unit_t unit, adc_channel_t channel,
                    AdcHandles &handles) {
  adc_oneshot_unit_init_cfg_t init = {
    .unit_id = ADC_UNIT,
      .ulp_mode = ADC_ULP_MODE_DISABLE
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init, &handles.unitHandle));

  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
  };

  ESP_ERROR_CHECK(
		  adc_oneshot_config_channel(handles.unitHandle, channel, &config));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!handles.calibrated) {
    ESP_LOGD("ADC", "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT,
      .chan = channel,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handles.caliHandle);
    if (ret == ESP_OK) {
      handles.calibrated = true;
    }
  }
#else
  //TODO support for another scheme
  ESP_LOGW("ADC", "No calibration scheme supported!");
#endif
}

