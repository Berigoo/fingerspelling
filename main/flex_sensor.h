#ifndef MAIN_FLEX_SENSOR
#define MAIN_FLEX_SENSOR

#define FLEX_SENSOR_COUNT 5
#define ADC_UNIT ADC_UNIT_1

#include <array>
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "hal/adc_types.h"
#include "utils.h"

/**
 * @brief intialize specific adc channel
 * @param channel -1 for intializing all of it
 * (incrementedd channel value) or @ref adc_channel_t struct
 */
void flex_init(int channel);
/**
 * @brief deinit contructedd unit handle and cali handle (if exists)
 * @param channel target channel
 */
void flex_clear(int channel);
float flex_read(adc_channel_t channel);
float flex_read_normalized(adc_channel_t channel);
float flex_normalize_voltage(float v);

#endif // MAIN_FLEX_SENSOR
