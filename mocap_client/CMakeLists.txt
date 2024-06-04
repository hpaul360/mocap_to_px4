cmake_minimum_required(VERSION 3.8)
project(mocap_client)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(mocap_interfaces REQUIRED)

# Get NatNet SDK
include(FetchContent)
FetchContent_Declare(natnet_sdk_content
  URL https://s3.amazonaws.com/naturalpoint/software/NatNetSDKLinux/ubuntu/NatNet_SDK_4.0_ubuntu.tar
)
FetchContent_MakeAvailable(natnet_sdk_content)

# Set SDK variables
set(NATNET_SDK_INCLUDE_DIR ${natnet_sdk_content_SOURCE_DIR}/include)
set(NATNET_SDK_LIBRARY ${natnet_sdk_content_SOURCE_DIR}/lib/libNatNet.so)

# Define target and include directories
add_executable(mocap_client src/natnet_client.cpp src/natnet_handler.cpp)
target_include_directories(mocap_client
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${mocap_interfaces_INCLUDE_DIRS}
  ${NATNET_SDK_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mocap_client
PUBLIC
  rclcpp::rclcpp
  ${mocap_interfaces_LIBRARIES}
  ${NATNET_SDK_LIBRARY}
)

# Install targets and files
install(TARGETS 
  mocap_client
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES ${NATNET_SDK_LIBRARY}
  DESTINATION lib
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
