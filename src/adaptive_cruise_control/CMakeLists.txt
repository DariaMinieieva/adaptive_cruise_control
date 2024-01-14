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
find_package(service_speed_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(lidar_node src/lidar_speed_node.cpp)
add_executable(main_cruise_control src/main_cruise_control.cpp)

ament_target_dependencies(lidar_node rclcpp irobot_create_msgs sensor_msgs geometry_msgs nav_msgs service_speed_interfaces)
ament_target_dependencies(main_cruise_control rclcpp irobot_create_msgs sensor_msgs geometry_msgs nav_msgs service_speed_interfaces)

target_include_directories(lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(main_cruise_control PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(main_cruise_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS lidar_node main_cruise_control
  DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/road_detect_main.py
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
