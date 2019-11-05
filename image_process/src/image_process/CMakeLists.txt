cmake_minimum_required(VERSION 2.8.3)
project(image_process)


add_compile_options(-std=c++11)

SET(CMAKE_MODULE_PATH
      ${CMAKE_MODULE_PATH}
      "${PROJECT_SOURCE_DIR}/cmake"
      )

set(PACKAGE_DEPENDENCIES
  eigen_conversions
  geometry_msgs
  message_runtime
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  darknet_ros_msgs
  cv_bridge  
  image_transport
  tf
  tf2
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(PCL 1.4 REQUIRED)  
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(DeepSort REQUIRED)
if(DeepSort_FOUND)
	message("===========================================================")
else()
	message(STATUS "++++++++++++++++++++++++++++++++++++++++++++++++++++++")
endif()

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS src
)


include_directories(${catkin_INCLUDE_DIRS} ${LUA_INCLUDE_DIR}
${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${DeepSort_INCLUDE_DIRS}
)

add_executable(image_process 
		src/main.cpp
		src/bbox_2d.h
		src/bbox_2d.cpp
		src/img_detection.h
		src/img_detection.cpp
		src/image_track.cpp
		src/image_track.h
		src/tracker_base.cpp
		src/tracker_base.h
		src/track_type.h
		src/ukf.cpp
		src/ukf.h
		src/multi_object_tracking.h
		src/multi_object_tracking.cpp
)


target_link_libraries(image_process ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${DeepSort_LIBRARIES} glog)

