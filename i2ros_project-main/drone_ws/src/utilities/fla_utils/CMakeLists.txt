cmake_minimum_required(VERSION 2.8.3)
project(fla_utils)

find_package(catkin REQUIRED
  geometry_msgs
  sensor_msgs
  tf
  laser_geometry
  fla_msgs
  nodelet
  cv_bridge
  image_transport
  )

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(utils REQUIRED)

add_definitions("-std=c++11")

catkin_package(
  INCLUDE_DIRS include src ${utils_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
  DEPENDS utils
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${utils_INCLUDE_DIRS}
  )

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
add_dependencies(laserscan_to_pointcloud fla_msgs_generate_messages_cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

add_executable(interpolate_imu src/interpolate_imu_node.cc)
target_link_libraries(interpolate_imu ${catkin_LIBRARIES})

add_library(median_filter_nodelet src/median_filter_nodelet.cpp)
target_link_libraries(median_filter_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

if(CATKIN_ENABLE_TESTING)
  # gtest
  enable_testing()

  catkin_add_gtest(${PROJECT_NAME}_test
    test/median_heap_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test median_filter_nodelet gtest ${catkin_LIBRARIES} ${GCOV_LIBRARY})
  target_compile_options(${PROJECT_NAME}_test PUBLIC ${${PROJECT_NAME}_COMPILER_FLAGS})
endif() # CATKIN_ENABLE_TESTING
