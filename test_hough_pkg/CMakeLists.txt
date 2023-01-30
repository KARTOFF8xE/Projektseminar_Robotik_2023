cmake_minimum_required(VERSION 3.5)
project(test_hough_pkg)

# Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${OpenCV_LIBRARY_DIRS})

ADD_EXECUTABLE(${PROJECT_NAME} src/main.cpp)
add_definitions(-O3)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
