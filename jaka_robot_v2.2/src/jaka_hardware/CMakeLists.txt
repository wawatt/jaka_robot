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

include_directories(
  include/jaka_hardware
)
link_directories(
  lib
)

add_library(
  jaka_hardware
  SHARED
  src/jaka_hardware_interface.cpp
)
target_include_directories(
  jaka_hardware
  PRIVATE
  include
  # ${xarm_sdk_INCLUDE_DIRS}
  # ${xarm_api_INCLUDE_DIRS}
  # ${ament_INCLUDE_DIRS}
)

ament_target_dependencies(
  jaka_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(jaka_hardware PUBLIC jakaAPI)

# message(STATUS "xarm_sdk_INCLUDE_DIRS: ${xarm_sdk_INCLUDE_DIRS}")
# message(WARNING "xarm_api_INCLUDE_DIRS: ${xarm_api_INCLUDE_DIRS}")
# message(WARNING "xarm_sdk_LIBRARIES: ${xarm_sdk_LIBRARIES}")
# message(WARNING "xarm_api_LIBRARIES: ${xarm_api_LIBRARIES}")

pluginlib_export_plugin_description_file(
  hardware_interface jaka_hardware_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS jaka_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES lib/jakaAPI.dll
  DESTINATION lib/${PROJECT_NAME}
)  

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_export_include_directories(include)
ament_export_libraries(jaka_hardware)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
