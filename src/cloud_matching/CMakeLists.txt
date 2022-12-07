set(SOURCES
  converter.cpp
  map_node.cpp
  ndt_matcher.cpp
  # phaser_matcher.cpp
  gicp_matcher.cpp
  g2o_optimizer.cpp
  local_map_maker.cpp
  global_map_maker.cpp
  node.cpp
  main.cpp
  )

set(MATCHER_TEST_SOURCES
  ndt_matcher.cpp
  converter.cpp
  gicp_matcher.cpp
  # phaser_matcher.cpp
  matcher_test.cpp
  )

set(OPTIMIZER_TEST_SOURCES
  converter.cpp
  g2o_optimizer.cpp
  optimizer_test.cpp
  )

add_executable(
  ${PROJECT_NAME}
  ${SOURCES}
)

add_library(
  matcher_lib
  ${MATCHER_TEST_SOURCES}
  )

add_executable(
  match_tester
  ${MATCHER_TEST_SOURCES}
  )

add_executable(
  optimize_tester
  ${OPTIMIZER_TEST_SOURCES}
  )

set(G2O_LIBS
  /opt/ros/noetic/lib/libg2o_types_slam3d.so
  /opt/ros/noetic/lib/libg2o_core.so
  /opt/ros/noetic/lib/libg2o_solver_cholmod.so
  /opt/ros/noetic/lib/libg2o_solver_csparse.so
  /opt/ros/noetic/lib/libg2o_solver_pcg.so
  )

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_LIBS}
  )

target_link_libraries(
  match_tester
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

target_link_libraries(
  optimize_tester
  ${catkin_LIBRARIES}
  ${G2O_LIBS}
  )

target_link_libraries(
  matcher_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
