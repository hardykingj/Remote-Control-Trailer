set(srcs "Adafruit_LIS3DH.cpp")

set(requires "bluepad32" "arduino")

string(REPLACE " -Werror" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    set_source_files_properties("Adafruit_LIS3DH.cpp" PROPERTIES
        COMPILE_FLAGS "-Wno-unused-value")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "src"
                    REQUIRES ${requires})