# - Try to find mex_conversion_utils header files
#
# Once done this will define
#
# MEX_CONVERSION_UTILS

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
set(mex_conversion_utils_INCLUDE_DIR "${PACKAGE_PREFIX_DIR}/mex_conversion_utils/include")
set(mex_conversion_utils_LIBRARY "")
