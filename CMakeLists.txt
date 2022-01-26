cmake_minimum_required(VERSION 3.1)
project(merge_maps_3d)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_CATKIN_DEPS
    pcl_conversions
    pcl_ros
    roscpp
    ndt_omp
    phaser_core
    fast_gicp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    std_srvs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPS}
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PROJECT_CATKIN_DEPS}
  DEPENDS
)

add_definitions(${catkin_DEFINITIONS})

include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS}
  SYSTEM ${PCL_INCLUDE_DIRS}
  )

add_subdirectory(src/${PROJECT_NAME})

########################################################
# Installs
########################################################

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
