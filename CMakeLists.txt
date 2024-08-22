cmake_minimum_required(VERSION 3.5)
project(BRoCo LANGUAGES CXX)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp_components REQUIRED)
#find_package(std_msgs REQUIRED)

# Add include directories
include_directories(
  include
  ${PROJECT_BINARY_DIR}/include
  include/BRoCo
  include/Build
  include/Protocol
)

# Find all source files in the source directory
file(GLOB BROCO_SRC
    "src/*.cpp"
)

# Create a shared library
add_library(BRoCo SHARED
    ${BROCO_SRC}
)

target_include_directories(BRoCo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Build>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Protocol>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/BRoCo>
)

ament_export_targets(BRoCo HAS_LIBRARY_TARGET)

# Link with ROS2 libraries
ament_target_dependencies(BRoCo
  "rclcpp"
)

install(DIRECTORY include/BRoCo
  DESTINATION include
)

# Install library and header files
install(TARGETS BRoCo
  EXPORT BRoCo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Add environment hooks

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()