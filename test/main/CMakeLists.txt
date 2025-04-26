idf_component_register(SRCS "unit_test_test.cpp" "../../main/utils.cpp"
                    INCLUDE_DIRS "." "../../main"
		    REQUIRES unity driver mpu60x0 main)
