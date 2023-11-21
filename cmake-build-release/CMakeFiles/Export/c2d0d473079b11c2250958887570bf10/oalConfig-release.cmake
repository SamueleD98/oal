#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "oal" for configuration "Release"
set_property(TARGET oal APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(oal PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liboal.so"
  IMPORTED_SONAME_RELEASE "liboal.so"
  )

list(APPEND _cmake_import_check_targets oal )
list(APPEND _cmake_import_check_files_for_oal "${_IMPORT_PREFIX}/lib/liboal.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
