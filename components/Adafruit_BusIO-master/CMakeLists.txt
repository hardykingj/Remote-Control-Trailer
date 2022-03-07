# Adafruit Bus IO Library
# https://github.com/adafruit/Adafruit_BusIO
# MIT License

cmake_minimum_required(VERSION 3.5)

set(srcs "Adafruit_BusIO_Register.cpp")
set(srcs "Adafruit_I2CDevice.cpp")
set(srcs "Adafruit_SPIDevice.cpp")

set(requires "bluepad32" "arduino")

string(REPLACE " -Werror" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties("Adafruit_BusIO_Register.cpp" PROPERTIES
        COMPILE_FLAGS "-Wno-unused-value")
endif()

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties("Adafruit_I2CDevice.cpp" PROPERTIES
        COMPILE_FLAGS "-Wno-unused-value")
endif()

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties("Adafruit_SPIDevice.cpp" PROPERTIES
        COMPILE_FLAGS "-Wno-unused-value")
endif()

idf_component_register(SRCS ${srcs} 
                       INCLUDE_DIRS "src"
                       REQUIRES ${requires})

project(Adafruit_BusIO)
