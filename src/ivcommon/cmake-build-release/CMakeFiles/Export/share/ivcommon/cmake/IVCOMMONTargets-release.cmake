#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ivcommon" for configuration "Release"
set_property(TARGET ivcommon APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ivcommon PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libivcommon.so"
  IMPORTED_SONAME_RELEASE "libivcommon.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ivcommon )
list(APPEND _IMPORT_CHECK_FILES_FOR_ivcommon "${_IMPORT_PREFIX}/lib/libivcommon.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
