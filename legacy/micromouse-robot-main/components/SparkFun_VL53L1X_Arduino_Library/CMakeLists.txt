# SparkFun VL53L1X Arduino library wrapped as an ESP-IDF component

file(GLOB_RECURSE SRC_FILES
    "SparkFun_VL53L1X_Arduino_Library-master/src/*.cpp"
)

idf_component_register(
    SRCS
        ${SRC_FILES}
    INCLUDE_DIRS
        "SparkFun_VL53L1X_Arduino_Library-master/include"
        "SparkFun_VL53L1X_Arduino_Library-master/src"
    REQUIRES
        espressif__arduino-esp32
)
