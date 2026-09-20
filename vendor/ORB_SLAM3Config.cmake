include(CMakeFindDependencyMacro)

get_filename_component(ORB_SLAM3_PREFIX
  "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

find_dependency(Eigen3 REQUIRED)
find_dependency(OpenCV REQUIRED)
find_dependency(Boost CONFIG REQUIRED COMPONENTS serialization)
find_dependency(OpenSSL REQUIRED)

if(NOT TARGET ORB_SLAM3::DBoW2)
  add_library(ORB_SLAM3::DBoW2 SHARED IMPORTED)
  set_target_properties(ORB_SLAM3::DBoW2 PROPERTIES
    IMPORTED_LOCATION "${ORB_SLAM3_PREFIX}/lib/libDBoW2.so"
    INTERFACE_INCLUDE_DIRECTORIES
      "${ORB_SLAM3_PREFIX}/include/orbslam3/Thirdparty/DBoW2")
endif()

if(NOT TARGET ORB_SLAM3::g2o)
  add_library(ORB_SLAM3::g2o SHARED IMPORTED)
  set_target_properties(ORB_SLAM3::g2o PROPERTIES
    IMPORTED_LOCATION "${ORB_SLAM3_PREFIX}/lib/libg2o.so"
    INTERFACE_INCLUDE_DIRECTORIES
      "${ORB_SLAM3_PREFIX}/include/orbslam3/Thirdparty/g2o")
endif()

if(NOT TARGET ORB_SLAM3::ORB_SLAM3)
  add_library(ORB_SLAM3::ORB_SLAM3 SHARED IMPORTED)
  set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
    IMPORTED_LOCATION "${ORB_SLAM3_PREFIX}/lib/libORB_SLAM3.so"
    INTERFACE_INCLUDE_DIRECTORIES
      "${ORB_SLAM3_PREFIX}/include/orbslam3;${ORB_SLAM3_PREFIX}/include/orbslam3/CameraModels;${ORB_SLAM3_PREFIX}/include/orbslam3/Thirdparty/Sophus"
    INTERFACE_LINK_LIBRARIES
      "ORB_SLAM3::DBoW2;ORB_SLAM3::g2o;Boost::serialization;OpenSSL::Crypto")
endif()

set(ORB_SLAM3_FOUND TRUE)
