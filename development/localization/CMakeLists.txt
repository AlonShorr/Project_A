cmake_minimum_required(VERSION 3.16)

set(
  EXTRA_COMPONENT_DIRS
  "${CMAKE_CURRENT_LIST_DIR}/../components"
  "${CMAKE_CURRENT_LIST_DIR}/../../legacy/micromouse-robot-main/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(localization)
