cmake_minimum_required(VERSION 2.8.3)
project(burger_clean)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  burger_arm_bringup
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(burger_clean src/burger_clean.cpp)
target_link_libraries(burger_clean ${catkin_LIBRARIES})
add_dependencies(burger_clean 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(sample src/sample.cpp)
target_link_libraries(sample ${catkin_LIBRARIES})
add_dependencies(sample 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
