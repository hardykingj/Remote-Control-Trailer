# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# See: https://gitlab.com/ricardoquesada/bluepad32/-/issues/9
set(ENV{BLUEPAD32_ARDUINO} TRUE)
set(ENV{ESP32_Servo} TRUE)
set(ENV{Adafruit_Sensor} TRUE)
set(ENV{DHT-sensor-library} TRUE)
set(ENV{Adafruit_BusIO-master} TRUE)
set(ENV{Adafruit_LIS3DH} TRUE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(RC_Car)
