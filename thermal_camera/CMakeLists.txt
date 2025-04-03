cmake_minimum_required(VERSION 3.5)
project(thermal_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)            # hard codes link paths into RUNPATH of installed binary to avoid need for LD_LIBRARY_PATH
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--disable-new-dtags")  # use RPATH instead of RUNPATH as RUNPATH is not searched for indirect dependencies

set(CMAKE_C_FLAGS "-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0")
set(CMAKE_CXX_FLAGS "-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(custom_msgs REQUIRED)

set(msg_files
  "msg/TemperatureMatrix.msg")

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs)

# add executable
set(NODE_NAME "thermal_camera_node")
file(GLOB MY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)
add_executable(${NODE_NAME} ${MY_SOURCES})

# specify include paths and dependencies
target_include_directories(${NODE_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" )
  find_library(IR_LIB
    NAMES irdirectsdk
    HINTS lib/libirimager/arm64)
  target_include_directories(${NODE_NAME} PRIVATE include/libirimager/arm64)

elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
  find_library(IR_LIB
    NAMES irdirectsdk
    HINTS lib/libirimager/amd64)
  target_include_directories(${NODE_NAME} PRIVATE include/libirimager/amd64)

else()
  message("Thermal Camera SDK API CPU architecture not supported.")

endif()

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${NODE_NAME} ${IR_LIB} ${OpenCV_LIBRARIES} yaml-cpp udev glut pthread usb-1.0 ${cpp_typesupport_target})
ament_target_dependencies(${NODE_NAME} rclcpp rclcpp_lifecycle sensor_msgs custom_msgs)

# install target at destination
install(
  TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# To make use of static library
install(
  FILES ${IR_LIB} 
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
