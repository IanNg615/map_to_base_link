cmake_minimum_required(VERSION 3.5)
project(map_to_base_link)

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
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

#find_package(rosidl_default_generators REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}

  
#add executables

add_executable(map_to_base_link src/map_to_base_link.cpp)
ament_target_dependencies(map_to_base_link std_msgs tf2_msgs geometry_msgs rclcpp tf2 tf2_ros)
  
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

install(TARGETS
  map_to_base_link
  DESTINATION lib/${PROJECT_NAME})
  
install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
