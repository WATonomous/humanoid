#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenXR::openxr_loader" for configuration "Debug"
set_property(TARGET OpenXR::openxr_loader APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(OpenXR::openxr_loader PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libopenxr_loader.so"
  IMPORTED_SONAME_DEBUG "libopenxr_loader.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenXR::openxr_loader )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenXR::openxr_loader "${_IMPORT_PREFIX}/lib/libopenxr_loader.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
