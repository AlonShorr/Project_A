set(LEGACY_MAIN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../legacy/micromouse-robot-main/main")
set(EIGEN_INCLUDE_DIR "${CMAKE_BINARY_DIR}/esp-idf/espressif__eigen/eigen-build/install/include/eigen3")

idf_component_register(
  SRCS
    "${LEGACY_MAIN_DIR}/distance_sensor.cpp"
    "${LEGACY_MAIN_DIR}/kalman_filter.cpp"
    "${LEGACY_MAIN_DIR}/motion_model.cpp"
    "${LEGACY_MAIN_DIR}/motor.cpp"
    "${LEGACY_MAIN_DIR}/periodic_caller.cpp"
    "${LEGACY_MAIN_DIR}/pid_controller.cpp"
    "${LEGACY_MAIN_DIR}/segment.cpp"
  INCLUDE_DIRS
    "${LEGACY_MAIN_DIR}"
    "${EIGEN_INCLUDE_DIR}"
  REQUIRES
    eigen
    espressif__arduino-esp32
    SparkFun_I2C_Mux_Arduino_Library
    SparkFun_VL53L1X_Arduino_Library
    misc_utils
    maze_solver
    bdc_motor
    led_strip
    pid_ctrl
)
