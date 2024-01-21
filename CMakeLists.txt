cmake_minimum_required(VERSION 3.0.2)
project(mapper)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sgtdv_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp
    CATKIN_DEPENDS std_msgs   
    CATKIN_DEPENDS geometry_msgs 
    CATKIN_DEPENDS sgtdv_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp src/${PROJECT_NAME}_node.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})