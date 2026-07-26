#----------------------------------------------------------------
# Generated CMake target import file for configuration "None".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "realsense2::rsutils" for configuration "None"
set_property(TARGET realsense2::rsutils APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(realsense2::rsutils PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librsutils.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::rsutils )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::rsutils "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librsutils.a" )

# Import target "realsense2::realsense-file" for configuration "None"
set_property(TARGET realsense2::realsense-file APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(realsense2::realsense-file PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "C;CXX"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librealsense-file.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::realsense-file )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::realsense-file "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librealsense-file.a" )

# Import target "realsense2::rs_lz4" for configuration "None"
set_property(TARGET realsense2::rs_lz4 APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(realsense2::rs_lz4 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "C"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librs_lz4.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::rs_lz4 )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::rs_lz4 "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librs_lz4.a" )

# Import target "realsense2::sqlite3_lib" for configuration "None"
set_property(TARGET realsense2::sqlite3_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(realsense2::sqlite3_lib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NONE "C"
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libsqlite3_lib.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::sqlite3_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::sqlite3_lib "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libsqlite3_lib.a" )

# Import target "realsense2::realsense2" for configuration "None"
set_property(TARGET realsense2::realsense2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NONE)
set_target_properties(realsense2::realsense2 PROPERTIES
  IMPORTED_LOCATION_NONE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librealsense2.so.2.58.2"
  IMPORTED_SONAME_NONE "librealsense2.so.2.58"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense2::realsense2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense2::realsense2 "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/librealsense2.so.2.58.2" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
