cmake_minimum_required(VERSION 3.8)
project(force_dimension)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# *****************************************************************************
# Find standard dependencies.
# *****************************************************************************

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# *****************************************************************************
# Find the Force Dimension SDK dependency.
# *****************************************************************************

# Ensure that the Force Dimension SDK can be found, if the environmental 
# variable is set.
if(DEFINED ENV{ForceDimensionSDK_DIR})
    set(ForceDimensionSDK_DIR $ENV{ForceDimensionSDK_DIR})
endif()

# Find the SDK.
find_package(ForceDimensionSDK REQUIRED)

# If the import of the Force Dimension SDK failed, then issue a message.
if(NOT ForceDimensionSDK_FOUND)
    message(ERROR "Set Force Dimension SDK base path via the environmental variable ForceDimensionSDK_DIR, or via the argument:  cmake -D ForceDimensionSDK_DIR=/path/to/sdk.")
endif()

# *****************************************************************************
# Add the Force Dimension node dependency.
# *****************************************************************************

add_executable(node src/entry_point.cpp src/node.cpp src/log.cpp src/publish.cpp src/subscribe.cpp)
ament_target_dependencies(node rclcpp geometry_msgs ForceDimensionSDK)

# *****************************************************************************
# Install.
# *****************************************************************************

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME})

# *****************************************************************************
# Install launch files.
# *****************************************************************************

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

# *****************************************************************************
# Testing.
# *****************************************************************************

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# *****************************************************************************
# Finish.
# *****************************************************************************

ament_package()

