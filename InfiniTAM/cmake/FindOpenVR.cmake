# - Try to find OpenVR
# Once done this will define
#  OpenVR_FOUND - System has OpenVR
#  OpenVR_INCLUDE_DIRS - The OpenVR include directories
#  OpenVR_LIBRARIES - The libraries needed to use LibXml2
#  OpenVR - A target to link against with CMake >= 3.0

# If OpenVR_ROOT is specified, search in it first.
if (OpenVR_ROOT)
    set(_OpenVR_SEARCH_ROOT PATHS ${OpenVR_ROOT} NO_DEFAULT_PATHS)
    list(APPEND _OpenVR_SEARCHES _OpenVR_SEARCH_ROOT)
endif()

# Normal search
set(_OpenVR_SERACH_NORMAL)
list(APPEND _OpenVR_SEARCHES _OpenVR_SERACH_NORMAL)

foreach(search ${_OpenVR_SEARCHES})
   find_path(OpenVR_INCLUDE_DIR NAMES openvr.h ${${search}}
     PATH_SUFFIXES include headers)
endforeach()

foreach(search ${_OpenVR_SEARCHES})
   find_library(OpenVR_LIBRARY NAMES openvr_api ${${search}}
     PATH_SUFFIXES
       lib
       # Other paths for the precompiled version of OpenVR
       lib/linux64 lib/win64)
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OpenVR
    REQUIRED_VARS OpenVR_INCLUDE_DIR OpenVR_LIBRARY)

if (OpenVR_FOUND)
    # Allow the user to override OpenVR_DIRS
    if (NOT OpenVR_INCLUDE_DIRS)
        set(OpenVR_INCLUDE_DIRS ${OpenVR_INCLUDE_DIR})
    endif()

    # Allow the user to override OpenVR_LIBRARIES
    if (NOT OpenVR_LIBRARIES)
        set(OpenVR_LIBRARIES ${OpenVR_LIBRARY})
    endif()

    # Define CMake target
    if (NOT TARGET OpenVR::OpenVR)
        add_library(OpenVR::OpenVR UNKNOWN IMPORTED)
        set_target_properties(OpenVR::OpenVR PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${OpenVR_INCLUDE_DIRS}")
        set_property(TARGET OpenVR::OpenVR APPEND PROPERTY
            IMPORTED_LOCATION "${OpenVR_LIBRARIES}")
    endif()
endif()
