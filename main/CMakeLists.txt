set(srcs "main.c")

set(requires "bluepad32")

set(requires "ESP32Servo-master")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs "arduino_main.cpp")
    list(APPEND requires "bluepad32_arduino" "arduino")
endif()

#list(APPEND srcs "ESP32_Servo.cpp")
list(APPEND requires "ESP32Servo-master")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
