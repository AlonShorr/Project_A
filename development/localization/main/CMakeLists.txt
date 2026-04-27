set(EIGEN_INCLUDE_DIR "${CMAKE_BINARY_DIR}/esp-idf/espressif__eigen/eigen-build/install/include/eigen3")

idf_component_register(
  SRCS
    localization.cpp
    main.cpp
  INCLUDE_DIRS
    .
    "${EIGEN_INCLUDE_DIR}"
  REQUIRES
    micromouse_interfaces
    micromouse_runtime
    eigen
    espressif__arduino-esp32
    SparkFun_I2C_Mux_Arduino_Library
    SparkFun_VL53L1X_Arduino_Library
    misc_utils
    maze_solver
)
