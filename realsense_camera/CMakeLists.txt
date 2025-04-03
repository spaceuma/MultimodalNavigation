cmake_minimum_required(VERSION 3.8)
project(realsense_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
find_package(custom_msgs REQUIRED)

set(msg_files
  "msg/DepthMatrix.msg")

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs)

set(NODE_NAME "realsense_camera_node")
file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_executable(${NODE_NAME} ${MY_SOURCES})

# specify include paths and dependencies
target_include_directories(${NODE_NAME} PRIVATE ${realsense2_INCLUDE_DIRS})

# Link libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${NODE_NAME} realsense2 yaml-cpp udev pthread usb-1.0 ${cpp_typesupport_target})
ament_target_dependencies(${NODE_NAME} rclcpp rclcpp_lifecycle custom_msgs sensor_msgs)


# install target at destination
install(
  TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

set_target_properties(${NODE_NAME} PROPERTIES
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
  BUILD_WITH_INSTALL_RPATH 1
)

# install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()