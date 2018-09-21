cmake_minimum_required(VERSION 2.8.3)
project(gazebo_fake_localization)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()


add_compile_options(-std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gazebo_fake_localization src/gazebo_fake_localization.cpp)
target_link_libraries(gazebo_fake_localization
   ${catkin_LIBRARIES}
)

add_dependencies(gazebo_fake_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
