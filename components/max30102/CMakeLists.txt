idf_component_register(SRCS "algorithm.c"
                      "max30102_api.c"
                      "i2c_api.c"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "."
)