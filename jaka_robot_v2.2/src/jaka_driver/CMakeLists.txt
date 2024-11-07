cmake_minimum_required(VERSION 3.5)
project(jaka_driver)


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


# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   jaka_msgs
#   moveit_msgs
#   actionlib
#   actionlib_msgs
# )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# catkin_package(
#   INCLUDE_DIRS include
# # LIBRARIES jaka_driver
#   CATKIN_DEPENDS roscpp rospy std_msgs jaka_msgs 
# #  DEPENDS system_lib
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

link_directories(
  lib
)

add_executable(jaka_driver src/jaka_driver.cpp)
target_include_directories(jaka_driver PRIVATE include)
add_executable(sdk_test src/sdk_test.cpp)
target_include_directories(sdk_test PRIVATE include)
add_executable(client src/client.cpp)
target_include_directories(client PRIVATE include)
add_executable(servoj_demo src/servoj_demo.cpp) 
target_include_directories(servoj_demo PRIVATE include)

ament_target_dependencies(jaka_driver rclcpp std_msgs std_srvs jaka_msgs sensor_msgs geometry_msgs moveit_msgs)
ament_target_dependencies(sdk_test rclcpp)
ament_target_dependencies(client rclcpp std_msgs jaka_msgs moveit_msgs)
ament_target_dependencies(servoj_demo rclcpp std_msgs jaka_msgs)

target_link_libraries(jaka_driver jakaAPI)
target_link_libraries(sdk_test jakaAPI)
target_link_libraries(servoj_demo jakaAPI)

install(TARGETS
  jaka_driver 
  sdk_test 
  client 
  servoj_demo
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