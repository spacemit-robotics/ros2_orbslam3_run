cmake_minimum_required(VERSION 3.16)
project(orbslam3_run)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

option(ORB_SLAM3_USE_RVV "Build the ROS nodes for SpacemiT RVV 1.0" ON)
set(ORB_SLAM3_PREFIX "/opt/orbslam3" CACHE PATH
  "ORB-SLAM3 and Pangolin installation prefix")
set(SPACEMIT_OPENCV_PREFIX "/opt/opencv-spacemit" CACHE PATH
  "SpacemiT RVV OpenCV installation prefix")

if(ORB_SLAM3_USE_RVV)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^riscv")
    message(FATAL_ERROR
      "ORB_SLAM3_USE_RVV defaults to ON and requires RISC-V; "
      "configure with -DORB_SLAM3_USE_RVV=OFF on other architectures")
  endif()
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "The default RVV flags require a GNU C++ compiler")
  endif()
  list(PREPEND CMAKE_PREFIX_PATH
    "${ORB_SLAM3_PREFIX}"
    "${SPACEMIT_OPENCV_PREFIX}")
  if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "${SPACEMIT_OPENCV_PREFIX}/lib/cmake/opencv4"
      CACHE PATH "OpenCV package configuration directory")
  endif()
  set(ORB_SLAM3_RUNTIME_RPATH
    "${ORB_SLAM3_PREFIX}/lib;${SPACEMIT_OPENCV_PREFIX}/lib")
else()
  list(PREPEND CMAKE_PREFIX_PATH "${ORB_SLAM3_PREFIX}")
  set(ORB_SLAM3_RUNTIME_RPATH "${ORB_SLAM3_PREFIX}/lib")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin CONFIG REQUIRED)
find_package(ORB_SLAM3 CONFIG REQUIRED)

add_executable(orbslam3_odometry_node src/orbslam3_odometry_node.cpp)
if(ORB_SLAM3_USE_RVV)
  target_compile_options(orbslam3_odometry_node PRIVATE
    -march=rv64gcv_zvl256b -mrvv-vector-bits=zvl)
  target_compile_definitions(orbslam3_odometry_node PRIVATE
    EIGEN_RISCV64_USE_RVV10)
endif()
target_include_directories(orbslam3_odometry_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS})
ament_target_dependencies(orbslam3_odometry_node
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs cv_bridge
  message_filters tf2 tf2_ros)
target_link_libraries(orbslam3_odometry_node
  ORB_SLAM3::ORB_SLAM3
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES})
set_target_properties(orbslam3_odometry_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_RUNTIME_RPATH}"
  INSTALL_RPATH "${ORB_SLAM3_RUNTIME_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH FALSE)

add_executable(orbslam3_slam_node src/orbslam3_slam_node.cpp)
if(ORB_SLAM3_USE_RVV)
  target_compile_options(orbslam3_slam_node PRIVATE
    -march=rv64gcv_zvl256b -mrvv-vector-bits=zvl)
  target_compile_definitions(orbslam3_slam_node PRIVATE
    EIGEN_RISCV64_USE_RVV10)
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Eigen 3.4 emits false-positive warnings for Transform::inverse at -O3.
  target_compile_options(orbslam3_slam_node PRIVATE -Wno-maybe-uninitialized)
endif()
target_include_directories(orbslam3_slam_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS})
ament_target_dependencies(orbslam3_slam_node
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs cv_bridge
  message_filters tf2 tf2_ros)
target_link_libraries(orbslam3_slam_node
  ORB_SLAM3::ORB_SLAM3
  ${OpenCV_LIBS}
  ${Pangolin_LIBRARIES})
set_target_properties(orbslam3_slam_node PROPERTIES
  BUILD_RPATH "${ORB_SLAM3_RUNTIME_RPATH}"
  INSTALL_RPATH "${ORB_SLAM3_RUNTIME_RPATH}"
  INSTALL_RPATH_USE_LINK_PATH FALSE)

install(TARGETS
  orbslam3_odometry_node
  orbslam3_slam_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch patches DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
