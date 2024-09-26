cmake_minimum_required(VERSION 3.5)
project(bumperbot_cpp_examples)

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

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

# declare executables and their dependencies 
add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

add_executable(simple_parametric src/simple_parametric.cpp)
ament_target_dependencies(simple_parametric rclcpp rcl_interfaces)

add_executable(simple_turtlesim_kinematics src/simple_turtlesim_kinematics.cpp)
ament_target_dependencies(simple_turtlesim_kinematics rclcpp turtlesim)

#install them
install(TARGETS
  simple_publisher
  simple_subscriber
  simple_parametric
  simple_turtlesim_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
