cmake_minimum_required(VERSION 3.1)

project(data_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

set(COMPILE_MODE "compile_basic")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  #pcl_conversions
#  pcl_ros
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  #nav_msgs
  message_generation
  visualization_msgs
)

#find_package(PCL REQUIRED )
find_package(OpenCV 4.2 REQUIRED )
find_package(Boost REQUIRED COMPONENTS thread serialization timer chrono)

catkin_package(
  INCLUDE_DIRS include
		thirdparty/json/
  DEPENDS #PCL
  CATKIN_DEPENDS
  std_msgs
  #nav_msgs
  geometry_msgs
  sensor_msgs

)

# include directories
include_directories( SYSTEM
	include
		thirdparty/json/
	${catkin_INCLUDE_DIRS}
#	${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
#	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

###########
## Build ##
###########



	# DataGenerator
	add_executable(${PROJECT_NAME}_data_generator src/data_generator.cpp )
	target_link_libraries(${PROJECT_NAME}_data_generator ${catkin_LIBRARIES} )
#endif()

install(DIRECTORY 
    include/${PROJECT_NAME}/ 
    thirdparty/json/
    rviz/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING
		PATTERN "*.h"
		PATTERN ".svn" EXCLUDE)