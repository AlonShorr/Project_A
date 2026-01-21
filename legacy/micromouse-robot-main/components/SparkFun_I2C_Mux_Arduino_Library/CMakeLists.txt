# SparkFun I2C Mux library as an ESP-IDF component

file(GLOB_RECURSE SRC_FILES "src/*.cpp")

idf_component_register(
    SRCS
        ${SRC_FILES}
    INCLUDE_DIRS
        "include"
    REQUIRES
        espressif__arduino-esp32
)
