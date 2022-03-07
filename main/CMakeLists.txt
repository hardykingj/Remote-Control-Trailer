set(srcs "main.c")

set(requires "bluepad32")

set(requires "ESP32Servo-master")

set(requires "Adafruit_Sensor-master")

set(requires "DHT-sensor-library-master")

set(requires "Adafruit_LIS3DH-master")

set(requires "Adafruit_BusIO-master")

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
if(DEFINED ENV{BLUEPAD32_ARDUINO})
    list(APPEND srcs "arduino_main.cpp")
    list(APPEND requires "bluepad32_arduino" "arduino")
endif()

list(APPEND requires "ESP32Servo-master")

list(APPEND requires "Adafruit_Sensor-master")

list(APPEND requires "DHT-sensor-library-master")

list(APPEND requires "Adafruit_LIS3DH-master")

list(APPEND requires "Adafruit_BusIO-master")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "."
                    REQUIRES "${requires}")
