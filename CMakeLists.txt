cmake_minimum_required(VERSION 2.8.3)
project(aloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  # std_msgs
  # visualization_msgs
  # roscpp
  # rospy
  # rosbag
  # image_transport
  # cv_bridge
  tf
)
# find_package( Boost REQUIRED COMPONENTS 
# timer )
 

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS} 
  # ${Boost_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs
  DEPENDS EIGEN3  
  INCLUDE_DIRS include
)

add_executable(PublishLidarData src/PublishLidarData.cpp)
target_link_libraries(PublishLidarData glog ${catkin_LIBRARIES})

add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})




