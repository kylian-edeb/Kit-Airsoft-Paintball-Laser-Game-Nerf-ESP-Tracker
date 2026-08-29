cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
"${CMAKE_CURRENT_LIST_DIR}/../../libraries"
"${CMAKE_CURRENT_LIST_DIR}/../shared/modules"
"${CMAKE_CURRENT_LIST_DIR}/../shared/protocols"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BEM)
