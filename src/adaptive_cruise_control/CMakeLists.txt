cmake_minimum_required(VERSION 3.8)
project(adaptive_cruise_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(cruise_control_node src/cruise_control_node.cpp)
ament_target_dependencies(cruise_control_node rclcpp irobot_create_msgs sensor_msgs geometry_msgs nav_msgs)

target_include_directories(cruise_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cruise_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS cruise_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/road_detect_main.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
