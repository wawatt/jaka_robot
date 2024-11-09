cmake_minimum_required(VERSION 3.5)
project(jaka_planner)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)

link_directories(
  lib
)

add_executable(jaka_control src/jaka_control.cpp)
# add_executable(moveit_test src/moveit_test.cpp)

target_include_directories(jaka_control PRIVATE include)
# target_include_directories(moveit_test PRIVATE include)

ament_target_dependencies(jaka_control 
  rclcpp rclcpp_action std_msgs jaka_msgs sensor_msgs control_msgs)
# ament_target_dependencies(moveit_test rclcpp std_msgs std_srvs jaka_msgs sensor_msgs geometry_msgs moveit_msgs)

target_link_libraries(jaka_control 
  jakaAPI
)

# target_link_libraries(moveit_test
#   jakaAPI
# )

install(TARGETS
  jaka_control 
  # moveit_test 
DESTINATION lib/${PROJECT_NAME})

install(
  FILES lib/jakaAPI.dll
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)  

ament_package()