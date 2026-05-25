cmake_minimum_required(VERSION 3.5)
project(franka_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs)

#################### LIBFRANKA PART ###############################

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED HINTS /docker_volume/libfranka/build)
find_package(Threads REQUIRED) 


add_executable(franka_node 
  src/main.cpp 
  src/franka_node.cpp
  include/franka_control/motion_generator.cpp
  include/franka_control/recorder.cpp
)

add_executable(franka_pose_node
  src/franka_pose_node.cpp
  include/franka_control/motion_generator.cpp
  include/franka_control/recorder.cpp
)

add_executable(franka_node_inference
  src/main_inference.cpp
  src/franka_node_inference.cpp
  include/franka_control/motion_generator.cpp
  include/franka_control/recorder.cpp
)

target_link_libraries(franka_node
  Franka::Franka 
  Eigen3::Eigen 
  Threads::Threads 
)

target_link_libraries(franka_pose_node
  Franka::Franka
  Eigen3::Eigen
  Threads::Threads
)

target_link_libraries(franka_node_inference
  Franka::Franka
  Eigen3::Eigen
  Threads::Threads
)

# include directories
target_include_directories(franka_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
)

target_include_directories(franka_pose_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
)

target_include_directories(franka_node_inference PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include
)

# link ROS libraries
ament_target_dependencies(
  franka_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  custom_msgs
)

ament_target_dependencies(
  franka_pose_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  custom_msgs
)

ament_target_dependencies(
  franka_node_inference
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  custom_msgs
)

# create library
install(TARGETS franka_node franka_pose_node franka_node_inference
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()


ament_export_libraries(${PROJECT_NAME})

ament_export_include_directories( 
  include
)
ament_export_dependencies(rclcpp geometry_msgs std_msgs sensor_msgs custom_msgs)
endif()

ament_package()
