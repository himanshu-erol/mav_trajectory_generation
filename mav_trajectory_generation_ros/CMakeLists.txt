cmake_minimum_required(VERSION 2.8.3)
project(mav_trajectory_generation_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)




#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/feasibility_analytic.cpp
  src/feasibility_base.cpp
  src/feasibility_recursive.cpp
  src/feasibility_sampling.cpp
  src/input_constraints.cpp
  src/ros_conversions.cpp
  src/ros_visualization.cpp
)

############
# BINARIES #
############
cs_add_executable(trajectory_sampler_node
  src/trajectory_sampler_node.cpp
)
target_link_libraries(trajectory_sampler_node ${PROJECT_NAME})

#########
# TESTS #
#########
catkin_add_gtest(test_feasibility
  test/test_feasibility.cpp
)
target_link_libraries(test_feasibility ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(run_test src/run_test.cpp)
target_link_libraries(run_test ${PROJECT_NAME} ${catkin_LIBRARIES})




find_package(octomap REQUIRED)
find_package(fcl REQUIRED)
find_package(OMPL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  rospy
  visualization_msgs
  interactive_markers
  tf
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs rospy visualization_msgs interactive_markers tf 
#  DEPENDS octomap PCL
)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})


add_executable(old_path_planning src/old_path_planning.cpp)
target_link_libraries(old_path_planning ${PROJECT_NAME}

   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
)

add_executable(cur_point_transformer src/cur_point_transformer.cpp)
target_link_libraries(cur_point_transformer ${PROJECT_NAME} ${catkin_LIBRARIES})




##########
# EXPORT #
##########
cs_install()
cs_export()
