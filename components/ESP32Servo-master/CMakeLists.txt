set(srcs "ESP32_Servo.cpp")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "include"
                    REQUIRES ${requires})