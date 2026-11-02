#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "escooter_dynamics::control" for configuration "Release"
set_property(TARGET escooter_dynamics::control APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(escooter_dynamics::control PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcontrol.so"
  IMPORTED_SONAME_RELEASE "libcontrol.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS escooter_dynamics::control )
list(APPEND _IMPORT_CHECK_FILES_FOR_escooter_dynamics::control "${_IMPORT_PREFIX}/lib/libcontrol.so" )

# Import target "escooter_dynamics::dynamics" for configuration "Release"
set_property(TARGET escooter_dynamics::dynamics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(escooter_dynamics::dynamics PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libdynamics.so"
  IMPORTED_SONAME_RELEASE "libdynamics.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS escooter_dynamics::dynamics )
list(APPEND _IMPORT_CHECK_FILES_FOR_escooter_dynamics::dynamics "${_IMPORT_PREFIX}/lib/libdynamics.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
