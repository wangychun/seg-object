cmake_minimum_required(VERSION 2.8)

project(cv_ft)                                    

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")        
add_definitions(-std=c++11)       
find_package(OpenCV REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)

include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(cv_ft opencv_st.cpp
			)

target_link_libraries(cv_ft ${OpenCV_LIBS})
