cmake_minimum_required(VERSION 3.8)
project(mvc01_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# libmodbus via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# Existing joystick-to-BLVR node
add_executable(joy_to_blvr src/joy_to_blvr.cpp)
target_compile_features(joy_to_blvr PUBLIC cxx_std_17)
target_include_directories(joy_to_blvr PUBLIC ${LIBMODBUS_INCLUDE_DIRS})
link_directories(${LIBMODBUS_LIBRARY_DIRS})
target_link_libraries(joy_to_blvr ${LIBMODBUS_LIBRARIES})
ament_target_dependencies(joy_to_blvr
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

# New cmd_vel-to-BLVR node
add_executable(blvr src/blvr.cpp)
target_compile_features(blvr PUBLIC cxx_std_17)
target_include_directories(blvr PUBLIC ${LIBMODBUS_INCLUDE_DIRS})
link_directories(${LIBMODBUS_LIBRARY_DIRS})
target_link_libraries(blvr ${LIBMODBUS_LIBRARIES})
ament_target_dependencies(blvr
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

# Install both
install(TARGETS joy_to_blvr blvr
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
