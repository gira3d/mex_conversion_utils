cmake_minimum_required(VERSION 2.8.3)
project(mex_conversion_utils)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(MATLAB QUIET)
find_package(Eigen3 REQUIRED)

include_directories(include
  ${MATLAB_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

if (MATLAB_FOUND)

  # MATLAB Macro definitions
  add_definitions(/DMATLAB_MEX_FILE)
  add_definitions(/DMX_COMPAT_32)

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION "include/${PROJECT_NAME}"
    FILES_MATCHING PATTERN "*.h"
  )

  install(DIRECTORY cmake/
    DESTINATION "share/${PROJECT_NAME}"
    FILES_MATCHING PATTERN "*.cmake"
  )
endif()
