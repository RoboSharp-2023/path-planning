cmake_minimum_required(VERSION 3.5)
project(path_planning)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(path_planning REQUIRED)

add_subdirectory(src/src)

#cpp
add_executable(pid src/PID_pos.cpp)
add_executable(serial_connection src/serial2.cpp)
add_executable(path_server src/path_server.cpp)

target_link_libraries(pid rrt_star)
target_include_directories(pid PUBLIC /home/naganaga/workspaces/gakurobo_ws/src/path_planning/src/include)

target_link_libraries(path_server rrt_star)
target_include_directories(path_server PUBLIC /home/naganaga/workspaces/gakurobo_ws/src/path_planning/src/include)

ament_target_dependencies(pid rclcpp std_msgs sensor_msgs)
ament_target_dependencies(serial_connection rclcpp std_msgs)
ament_target_dependencies(path_server rclcpp path_planning)

install(TARGETS 
  serial_connection 
  pid 
  path_server 
  DESTINATION lib/${PROJECT_NAME}
)

#launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

#python
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  script/ros_serial.py
  script/path_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

#ros2 msg
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/PathData.msg
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()