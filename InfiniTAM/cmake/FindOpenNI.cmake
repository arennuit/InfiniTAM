# - Find OpenNI
# This module defines
#  OpenNI_INCLUDE_DIR, where to find OpenNI include files
#  OpenNI_LIBRARIES, the libraries needed to use OpenNI
#  OpenNI_FOUND, If false, do not try to use OpenNI.
# also defined, but not for general use are
#  OpenNI_LIBRARY, where to find the OpenNI library.

set(OPEN_NI_ROOT "/usr/local" CACHE FILEPATH "Root directory of OpenNI2")

# Finally the library itself
find_library(OpenNI_LIBRARY
  NAMES OpenNI2
  PATHS "${OPEN_NI_ROOT}"
  PATH_SUFFIXES "Bin/x64-Release" "Lib" "Redist"
)

find_path(OpenNI_INCLUDE_DIR OpenNI.h
  PATHS "${OPEN_NI_ROOT}"
  PATH_SUFFIXES "Include")

# handle the QUIETLY and REQUIRED arguments and set JPEG_FOUND to TRUE if
# all listed variables are TRUE
#include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
#include(${CMAKE_MODULE_PATH}/FindPackageHandleStandardArgs.cmake)
find_package_handle_standard_args(OpenNI DEFAULT_MSG OpenNI_LIBRARY OpenNI_INCLUDE_DIR)

if(OPENNI_FOUND)
  set(OpenNI_LIBRARIES ${OpenNI_LIBRARY})
endif()

mark_as_advanced(OpenNI_LIBRARY OpenNI_INCLUDE_DIR)
