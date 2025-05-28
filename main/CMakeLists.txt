idf_component_register(SRCS "esp32-flex_sensors.cpp" "flex_sensor.cpp" "flex_sensor.h" "utils.cpp" "utils.h" "button.cpp" "button.h" "tf.cpp" "tf.h"
                    INCLUDE_DIRS "."
		    REQUIRES I2Cdev MPU6050 driver esp_timer esp_adc)
