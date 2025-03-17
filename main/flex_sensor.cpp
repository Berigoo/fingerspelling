#include "flex_sensor.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include <array>
#include <utility>

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


struct AdcUnitHandles {
  adc_unit_t unitId;
  adc_oneshot_unit_handle_t unitHandle;
};
typedef std::pair<AdcUnitHandles, bool> unit_context_t;

struct AdcHandles {
  unit_context_t *pUnitContext; // TODO
  adc_cali_handle_t caliHandle;
  adc_channel_t channel;
};
struct AdcHandlesInfo {
  bool isInitialized = false;
  bool isCalibrated = false;
};
typedef std::pair<AdcHandlesInfo, AdcHandles> flex_context_t;


static std::array<flex_context_t, FLEX_SENSOR_COUNT> s_adc_handles_lookup;
static std::array<unit_context_t, 2> s_adc_unit_handles{};

static void init_adc_channel(adc_channel_t channel, unit_context_t& handles, flex_context_t& flexHandle);

// void flex_init(int channel) {
//   int count = 0;
//   if (channel < 0) {
//     for (auto &e : s_adc_handles_lookup) {
//       init_adc_channel(ADC_UNIT, (adc_channel_t)(count+1), e);
//       count++;
//     }
//   } else {
//     init_adc_channel(ADC_UNIT, (adc_channel_t)channel, s_adc_handles_lookup[channel]);
//   }
// }

void *adc_unit_create(int id) {
  for (auto& e : s_adc_unit_handles) {
    if (!e.second) {
      adc_oneshot_unit_init_cfg_t init = {
	.unit_id = ADC_UNIT,
	.ulp_mode = ADC_ULP_MODE_DISABLE
      };

      ESP_ERROR_CHECK(adc_oneshot_new_unit(&init, &e.first.unitHandle));
      e.second = true;
      return &e;
    }
  }
  ESP_LOGE("ADC", "Failed to create adc_unit_context");
  return nullptr;
}

void *flex_create(void *unitHandle, int channel) {
  unit_context_t* unit = static_cast<unit_context_t*>(unitHandle);
  for (auto& e : s_adc_handles_lookup) {
    if (!e.first.isInitialized) {
      init_adc_channel((adc_channel_t)channel, *unit,
                       e);
      e.first.isInitialized = true;
      e.second.channel = (adc_channel_t)channel;
      e.second.pUnitContext = unit;
      ESP_LOGD("ADC", "Adc Oneshot create at: %d channel", (adc_channel_t)channel);            
      return &e;
    }
  }
  ESP_LOGE("ADC", "Failed to create flex_sensor_context");
  return nullptr;
}

float flex_read(void *handle) {
  int raw, mvoltage;
  float voltage = -1;
  flex_context_t *handles = static_cast<flex_context_t *>(handle);
  ESP_ERROR_CHECK(adc_oneshot_read(handles->second.pUnitContext->first.unitHandle, handles->second.channel, &raw));
  ESP_LOGD("ADC", "ADC%d Channel[%d] raw: %d", ADC_UNIT + 1, handles->second.channel, raw);
  if (handles->first.isCalibrated) {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(handles->second.caliHandle, raw, &mvoltage));
    voltage = mvoltage / 1000.0f;
    ESP_LOGD("ADC", "ADC%d Channel[%d] V: %f", ADC_UNIT+1, handles->second.channel, voltage);
    ESP_LOGD("ADC", "ADC%d Channel[%d] normalized: %f", ADC_UNIT + 1,
	     handles->second.channel, (normalization(voltage, VOLTAGE_MIN, VOLTAGE_MAX)));
  }
  
  return voltage;
}

void flex_clear(void *flexHandle) {
  flex_context_t* handles = static_cast<flex_context_t*>(flexHandle);
  if (handles->first.isCalibrated) {
    adc_cali_delete_scheme_curve_fitting(handles->second.caliHandle);
  }
}

void adc_unit_clear(void *handle) {
  unit_context_t *handles = static_cast<unit_context_t *>(handle);
  ESP_ERROR_CHECK(adc_oneshot_del_unit(handles->first.unitHandle));
}

// void flex_clear(int channel) {
//   if (channel < 0) {
//     for (auto &e : s_adc_handles_lookup) {
//       ESP_ERROR_CHECK(adc_oneshot_del_unit(e.unitHandle));
//       if (e.calibrated) {
//         adc_cali_delete_scheme_curve_fitting(e.caliHandle);
//       }
//     }
//   } else {
//     ESP_ERROR_CHECK(adc_oneshot_del_unit(s_adc_handles_lookup[channel].unitHandle));
//     if (s_adc_handles_lookup[channel].calibrated) {
//       adc_cali_delete_scheme_curve_fitting(s_adc_handles_lookup[channel].caliHandle);
//     }
//   }
// }

// float flex_read(adc_channel_t channel) {
//   int raw, mvoltage;
//   float voltage = -1;
//   AdcHandles* handles = &s_adc_handles_lookup[channel];
//   ESP_ERROR_CHECK(adc_oneshot_read(handles->unitHandle, channel, &raw));
//   ESP_LOGD("ADC", "ADC%d Channel[%d] raw: %d", ADC_UNIT + 1, channel, raw);
//   if (handles->calibrated) {
//     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(handles->caliHandle, raw, &mvoltage));
//     voltage = mvoltage / 1000.0f;
//     ESP_LOGD("ADC", "ADC%d Channel[%d] V: %f", ADC_UNIT+1, channel, voltage);
//     ESP_LOGD("ADC", "ADC%d Channel[%d] normalized: %f", ADC_UNIT + 1,
// 	     channel, (normalization(voltage, VOLTAGE_MIN, VOLTAGE_MAX)));
//   }
  
//   return voltage;
// }

// float flex_read_normalized(adc_channel_t channel) {
//   AdcHandles *handles = &s_adc_handles_lookup[channel];
//   if (handles->calibrated) {
//     int raw, mvoltage;
//     float voltage = -1, normalized;
  
//     ESP_ERROR_CHECK(adc_oneshot_read(handles->unitHandle, channel, &raw));
//     ESP_LOGD("ADC", "ADC%d Channel[%d] raw: %d", ADC_UNIT + 1, channel, raw);
  
//     ESP_ERROR_CHECK(adc_cali_raw_to_voltage(handles->caliHandle, raw, &mvoltage));
//     voltage = mvoltage / 1000.0f;
//     normalized = normalization(voltage, VOLTAGE_MIN, VOLTAGE_MAX);
    
//     ESP_LOGD("ADC", "ADC%d Channel[%d] V: %f", ADC_UNIT+1, channel, voltage);
//     ESP_LOGD("ADC", "ADC%d Channel[%d] normalized: %f", ADC_UNIT + 1, channel,
// 	     normalized);

//     return normalized;
//   } else {
//     ESP_LOGE("ADC", "ADC%d Channel[%d] not successfully calibrated", ADC_UNIT+1, channel);
//     return -1.0f;
//   }
// }

float flex_normalize_voltage(float v) {
  return normalization(v, VOLTAGE_MIN, VOLTAGE_MAX);
}


void init_adc_channel(adc_channel_t channel, // TODO
		      unit_context_t &handles, flex_context_t &flexhandle) {
  adc_oneshot_chan_cfg_t config = {
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
  };

  ESP_ERROR_CHECK(
		  adc_oneshot_config_channel(handles.first.unitHandle, channel, &config));

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  if (!flexhandle.first.isCalibrated) {
    ESP_LOGD("ADC", "calibration scheme version is %s", "Curve Fitting");
    adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = handles.first.unitId,
      .chan = channel,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t ret = adc_cali_create_scheme_curve_fitting(
        &cali_config, &flexhandle.second.caliHandle);
    if (ret == ESP_OK) {
      ESP_LOGD("ADC", "channel %d calibrated", channel);
      flexhandle.first.isCalibrated = true;
    }
  }
#else
  //TODO support for another scheme
  ESP_LOGW("ADC", "No calibration scheme supported!");
#endif
}

