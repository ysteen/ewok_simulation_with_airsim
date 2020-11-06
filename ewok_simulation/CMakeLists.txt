cmake_minimum_required(VERSION 3.0.2)
project(ewok_simulation)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_definitions(-std=c++11 -g -O3)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  mavros
  rospy
  std_msgs
)


include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   scripts/offb_node.py
 )

catkin_simple()

cs_add_executable(bspline_lee_position_controller_node src/bspline_lee_position_controller_node.cpp)
cs_add_executable(bspline_lee_position_controller_node_px4 src/bspline_lee_position_controller_node_px4.cpp)
cs_add_executable(trajectory_replanning_example src/trajectory_replanning_example.cpp)
cs_add_executable(trajectory_replanning_example_airsim src/trajectory_replanning_example_airsim.cpp)
target_link_libraries(trajectory_replanning_example ${CHOLMOD_LIBRARY} nlopt)
target_link_libraries(trajectory_replanning_example_airsim ${CHOLMOD_LIBRARY} nlopt)

cs_install()
cs_export()
