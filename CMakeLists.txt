cmake_minimum_required(VERSION 3.1)
project(cloud_matching)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_CATKIN_DEPS
    dynamic_reconfigure
    fast_gicp
    geometry_msgs
    ndt_omp
    pcl_conversions
    pcl_ros
    # phaser_core
    roscpp
    sensor_msgs
    std_srvs
    visualization_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${PROJECT_CATKIN_DEPS}
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  config/CloudMatching.cfg
  )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    matcher_lib
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

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

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
