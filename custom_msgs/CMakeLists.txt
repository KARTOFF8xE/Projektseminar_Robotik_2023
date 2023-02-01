cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(rosidl_default_generators REQUIRED)

#generate specified messages
set(MESSAGES "")
list(APPEND MESSAGES "msg/Distance.msg")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${MESSAGES}
  DEPENDENCIES builtin_interfaces std_msgs)

ament_package()
