cmake_minimum_required(VERSION 3.5)
project(test_pkg)

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(filters REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

set(SOURCES "")
list(APPEND SOURCES "src/node.cpp")
list(APPEND SOURCES "src/main.cpp")
list(APPEND SOURCES "src/src.cpp")

add_executable(test_pkg ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} filters::filters_lib)
add_definitions(-O3)
ament_target_dependencies(test_pkg rclcpp sensor_msgs)

install(TARGETS
  test_pkg
  DESTINATION lib/${PROJECT_NAME}
)

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install PARAM files
install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)

ament_package()
