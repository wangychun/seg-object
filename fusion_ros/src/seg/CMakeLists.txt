cmake_minimum_required(VERSION 2.8.3)
project(seg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(CMAKE_BUILD_TYPE "Release")

SET(CMAKE_MODULE_PATH
      ${CMAKE_MODULE_PATH}
      "${PROJECT_SOURCE_DIR}/cmake"
      )


find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	cv_bridge 
	std_msgs 
	image_transport
)                                                           
find_package(PCL 1.4 REQUIRED)  
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)



catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS src
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(seg src/main.cpp
		   src/gridmap.h
		   src/gridmap.cpp
                   src/pic_handle.cpp
 #                  src/get_data.h
 #                 src/get_data.cpp
			)

target_link_libraries(seg ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS})


