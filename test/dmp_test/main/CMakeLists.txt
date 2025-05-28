idf_component_register(SRCS "dmp_test.cpp" "MadgwickAHRS.cpp"
                    INCLUDE_DIRS "." 
		    REQUIRES driver I2Cdev MPU6050 esp_timer)
