cmake_minimum_required(VERSION 3.5)
project(jaka_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  realtime_tools
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

link_directories(
  lib
)

###########
## Build ##
###########

# Declare a C++ library
add_library(
  ${PROJECT_NAME} SHARED
  src/jaka_hardware_interface.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME}
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} jakaAPI.lib)

# message(STATUS "xarm_sdk_INCLUDE_DIRS: ${xarm_sdk_INCLUDE_DIRS}")
# message(WARNING "xarm_api_INCLUDE_DIRS: ${xarm_api_INCLUDE_DIRS}")
# message(WARNING "xarm_sdk_LIBRARIES: ${xarm_sdk_LIBRARIES}")
# message(WARNING "xarm_api_LIBRARIES: ${xarm_api_LIBRARIES}")

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES lib/jakaAPI.dll
  DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(
  hardware_interface jaka_hardware_plugins.xml)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
