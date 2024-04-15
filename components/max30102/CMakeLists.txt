idf_component_register(SRCS "algorithm.c"
                      "algorithm_by_RF.c"
                      "MAX30102.c"
                      "i2c_api.c"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_timer
)