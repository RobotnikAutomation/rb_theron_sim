cmake_minimum_required(VERSION 2.8.3)
project(rb_theron_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rcomponent
  robotnik_msgs
  roscpp
)

#add_message_files(
#  FILES
#  Message1.msg
#)

catkin_package(
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
)

add_compile_options(-std=c++11)

#   src/${PROJECT_NAME}/${PROJECT_NAME}.cpp

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# catkin_add_gtest(${PROJECT_NAME}-test test/test_${PROJECT_NAME}.cpp)

