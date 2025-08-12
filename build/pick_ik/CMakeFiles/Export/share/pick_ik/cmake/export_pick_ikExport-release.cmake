#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pick_ik::pick_ik_plugin" for configuration "Release"
set_property(TARGET pick_ik::pick_ik_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(pick_ik::pick_ik_plugin PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "fmt::fmt;moveit_core::moveit_kinematics_base;moveit_core::moveit_robot_state;rclcpp::rclcpp"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpick_ik_plugin.so"
  IMPORTED_SONAME_RELEASE "libpick_ik_plugin.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pick_ik::pick_ik_plugin )
list(APPEND _IMPORT_CHECK_FILES_FOR_pick_ik::pick_ik_plugin "${_IMPORT_PREFIX}/lib/libpick_ik_plugin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
