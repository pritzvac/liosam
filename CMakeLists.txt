cmake_minimum_required(VERSION 3.1.2)
cmake_policy(SET CMP0048 NEW)
project(liosam)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
# if (DEFINED ENV{PCL_CROSS_COMPILATION})
#   set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
# else()
#   set(PCL_CROSS_COMPILATION "false")
# endif()
# if(${PCL_CROSS_COMPILATION} STREQUAL "false")
#   message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
#   set(CMAKE_CXX_FLAGS "-std=c++17")
# else()
#   message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
# endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -pthread")
set(CMAKE_CXX_FLAGS_RELDEB "${CMAKE_CXX_FLAGS_RELDEB} -pthread")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  nodelet
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  mrs_lib
  mrs_msgs
  gtsam_playground
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread timer chrono)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  mrs_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM Boost

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
  visualization_msgs
  mrs_lib
  mrs_msgs
  gtsam_playground
)

# add definitions
add_definitions(
    ${PCL_DEFINITIONS}
    ${GTSAM_DEFINITIONS}
    ${OpenCV_DEFINITIONS}
  )

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
add_library(ImageProjection src/imageProjection.cpp)
add_dependencies(ImageProjection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  )
target_link_libraries(ImageProjection
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# Feature Association
add_library(FeatureExtraction src/featureExtraction.cpp)
add_dependencies(FeatureExtraction
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  )
target_link_libraries(FeatureExtraction
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# Mapping Optimization
add_library(MapOptimization src/mapOptimization.cpp)
add_dependencies(MapOptimization
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  )
target_compile_options(MapOptimization
  PRIVATE
  ${OpenMP_CXX_FLAGS}
  )
target_link_libraries(MapOptimization
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GTSAM_LIBRARIES}
  gtsam
  )

# IMU Preintegration
add_library(ImuPreintegration src/imuPreintegration.cpp)
add_dependencies(ImuPreintegration
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  )
target_link_libraries(ImuPreintegration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GTSAM_LIBRARIES}
  gtsam
  )

# MotorSpeed Preintegration
add_library(MotorSpeedPreintegration src/motorSpeedPreintegration.cpp)
add_dependencies(MotorSpeedPreintegration
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  )
target_link_libraries(MotorSpeedPreintegration
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GTSAM_LIBRARIES}
  gtsam
  )

# Transform Fusion
add_library(TransformFusion src/transformFusion.cpp)
add_dependencies(TransformFusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(TransformFusion
  ${catkin_LIBRARIES}
  )
