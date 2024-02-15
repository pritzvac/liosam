cmake_minimum_required(VERSION 3.1.2)
cmake_policy(SET CMP0048 NEW)
project(liosam)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -pthread")
set(CMAKE_CXX_FLAGS_RELDEB "${CMAKE_CXX_FLAGS_RELDEB} -pthread")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
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

set(LIBRARIES
  ImuPreintegration
  MapOptimization
  FeatureExtraction
  ImageProjection
  TransformFusion
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARIES}
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

# Transform Fusion
add_library(TransformFusion src/transformFusion.cpp)
add_dependencies(TransformFusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(TransformFusion
  ${catkin_LIBRARIES}
  )


## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


install(FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
