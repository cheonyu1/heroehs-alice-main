cmake_minimum_required(VERSION 2.8.3)
project(main_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  alice_msgs
  robotis_controller_msgs
  log_manager
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES game_communicator
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} src/${PROJECT_NAME}/game_communicator.cpp)
add_executable(${PROJECT_NAME}_node src/main_controller_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

