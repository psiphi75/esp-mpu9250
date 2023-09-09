idf_component_register(SRCS         "ak8963.c"
                                    "calibrate.c"
                                    "common.c"
                                    "i2c-easy.c"
                                    "mpu9250.c"
                       INCLUDE_DIRS "include"
                       REQUIRES driver)