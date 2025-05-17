#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LZ4::lz4_shared" for configuration ""
set_property(TARGET LZ4::lz4_shared APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(LZ4::lz4_shared PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblz4.so.1.10.0"
  IMPORTED_SONAME_NOCONFIG "liblz4.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS LZ4::lz4_shared )
list(APPEND _IMPORT_CHECK_FILES_FOR_LZ4::lz4_shared "${_IMPORT_PREFIX}/lib/liblz4.so.1.10.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
