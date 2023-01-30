cmake_minimum_required(VERSION 3.5)
project(lidar_curb_detection)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

link_directories(${OpenCV_LIBRARY_DIRS})

set(SOURCES "")
list(APPEND SOURCES "src/main.cpp")
list(APPEND SOURCES "src/node.cpp")
list(APPEND SOURCES "src/lidar_curb_detection.cpp")

add_library(${PROJECT_NAME}_lib "src/${PROJECT_NAME}.cpp" "include/${PROJECT_NAME}/${PROJECT_NAME}.hpp")

add_executable(${PROJECT_NAME} ${SOURCES}) # `pkg-config --libs opencv4`
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PROJECT_NAME}_lib)
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
add_definitions(-O3)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs custom_msgs)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
