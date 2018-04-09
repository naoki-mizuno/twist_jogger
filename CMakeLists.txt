cmake_minimum_required(VERSION 2.8.3)
project(twist_jogger)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_manipulation
  moveit_ros_move_group
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
)

add_library(twist_jogger
  src/twist_jogger.cpp include/twist_jogger.h
)

target_link_libraries(twist_jogger
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(twist_jogger_node
  src/twist_jogger_node.cpp
)

target_link_libraries(twist_jogger_node
  twist_jogger
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS twist_jogger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
