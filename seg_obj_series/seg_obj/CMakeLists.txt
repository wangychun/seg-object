cmake_minimum_required(VERSION 2.8)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

project(seg_obj)                                    

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")        
add_definitions(-std=c++11)                                                              
find_package(PCL 1.4 REQUIRED)  
find_package(OpenCV REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(seg_obj src/main.cpp
			src/gridmap.cpp
			src/gridmap.h
			src/pic_handle.cpp
			)

target_link_libraries(seg_obj ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
								${OpenCV_LIBS})
