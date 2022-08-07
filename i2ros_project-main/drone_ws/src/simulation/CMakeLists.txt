cmake_minimum_required(VERSION 2.8.3)
project(simulation)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rospy
	sensor_msgs
	std_msgs
	tf
	image_transport
        fla_utils
        fla_msgs
        cv_bridge
        message_generation
        mav_msgs
	)

catkin_package(CATKIN_DEPENDS
        mav_msgs
	)

include(ExternalProject)
ExternalProject_Add(libsocket
  GIT_REPOSITORY https://github.com/dermesser/libsocket
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_SHARED_LIBS=ON
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)

find_package(Eigen3)

link_directories(${CATKIN_DEVEL_PREFIX}/lib)
include_directories(${CATKIN_DEVEL_PREFIX}/include)
include_directories(${catkin_INCLUDE_DIRS} tcpimage)

add_subdirectory(tcpimage)

add_executable(unity_ros src/unity_ros.cpp)
add_executable(state_estimate_corruptor_node src/state_estimate_corruptor_node.cpp )
add_executable(w_to_unity src/w_to_unity.cpp)
add_executable(transform_broadcaster src/transform_broadcaster.cpp)
add_executable(time_server src/time_server.cpp)

target_link_libraries(unity_ros ${catkin_LIBRARIES} tcpstreamreader tcpimage)
target_link_libraries(state_estimate_corruptor_node  ${catkin_LIBRARIES})
target_link_libraries(w_to_unity ${catkin_LIBRARIES} socket++ -pthread)
target_link_libraries(transform_broadcaster ${catkin_LIBRARIES})
target_link_libraries(time_server ${catkin_LIBRARIES})

add_dependencies(w_to_unity libsocket ${catkin_EXPORTED_TARGETS})

