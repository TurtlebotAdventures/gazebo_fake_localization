cmake_minimum_required(VERSION 2.8.3)
project(gazebo_fake_localization)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

add_compile_options(-std=c++11)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_fake_localization
  CATKIN_DEPENDS roscpp tf2_ros tf2_geometry_msgs nav_msgs gazebo_msgs
)


add_library(gazebo_fake_localization
  src/gazebo_fake_localization.cpp
  src/direct_pose_gazebo_fake_localization.cpp
)

target_link_libraries(gazebo_fake_localization
   ${catkin_LIBRARIES}
)

# Add executable nodes
add_executable(gazebo_fake_localization_node src/gazebo_fake_localization_node.cpp)
add_executable(direct_pose_gazebo_fake_localization_node src/direct_pose_gazebo_fake_localization_node.cpp)

# Link executables to the gazebo_fake_localization library
target_link_libraries(gazebo_fake_localization_node
  gazebo_fake_localization
  ${catkin_LIBRARIES}
)

target_link_libraries(direct_pose_gazebo_fake_localization_node
  gazebo_fake_localization
  ${catkin_LIBRARIES}
)

add_dependencies(gazebo_fake_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
