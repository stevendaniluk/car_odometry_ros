cmake_minimum_required(VERSION 2.8.3)
project(car_odometry_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include car_odometry/include
  LIBRARIES car_odometry car_odometry_ros
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
  car_odometry/include
)

###########
## Build ##
###########

# Add our core odometry library
add_subdirectory(car_odometry)

# ROS wrappers library
add_library(car_odometry_ros
  src/car_odometry_ros.cpp
  src/conversions.cpp
)
target_link_libraries(car_odometry_ros
  ${catkin_LIBRARIES}
  car_odometry
)

#############
## Testing ##
#############

# Unit test for ros conversions
add_executable(test_car_odometry_ros_conversions test/test_car_odometry_ros_conversions.cpp)
add_test(NAME test_car_odometry_ros_conversions COMMAND test_car_odometry_ros_conversions)
target_link_libraries(test_car_odometry_ros_conversions
  gtest_main
  car_odometry_ros
)
