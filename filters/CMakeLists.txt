cmake_minimum_required(VERSION 3.5)
project(filters)

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
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})

link_directories(${OpenCV_LIBRARY_DIRS})

set(LIB_SOURCES "")
list(APPEND LIB_SOURCES "src/encoding_filter.cpp")
list(APPEND LIB_SOURCES "src/post_filters.cpp")

add_library(${PROJECT_NAME}_lib "include/${PROJECT_NAME}/filters.hpp" ${LIB_SOURCES})
ament_target_dependencies(${PROJECT_NAME}_lib sensor_msgs)
target_link_libraries(${PROJECT_NAME}_lib ${OpenCV_LIBRARIES})

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