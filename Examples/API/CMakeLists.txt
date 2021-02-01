cmake_minimum_required(VERSION 2.8.3)

project(ORB_SLAM3_API)

add_library(ORB_SLAM3_API SHARED
    ORBSLAM3_api.cpp
)

target_include_directories(ORB_SLAM3_API
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    INTERFACE
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(ORB_SLAM3_API
PRIVATE
    ${GLOG_LIBRARIES}
    ${EIGEN3_LIBS}
    ORB_SLAM3
)

add_subdirectory(test)
