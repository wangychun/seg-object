# Try to find DeepSort library
# Once done this will define
#  DeepSort_FOUND - if system found Part4 library
#  DeepSort_INCLUDE_DIRS - The Part4 include directories
#  DeepSort_LIBRARIES - The libraries needed to use Part4
#  DeepSort_DEFINITIONS - Compiler switches required for using Part4

#set (DeepSort_ROOT_DIR /home/zhanghm/Coding/deep_sort_cpp/install/)
set (DeepSort_ROOT_DIR /home/sarah/program/deep_sort_cpp/install/)
# Uncomment the following line to print which directory CMake is looking in.
MESSAGE(STATUS "DeepSort_ROOT_DIR: " ${DeepSort_ROOT_DIR})

find_path(DeepSort_INCLUDE_DIR 
		NAMES deep_sort/multi_object_track.h
		PATHS ${DeepSort_ROOT_DIR}/include
		ENV DEEPSORTROOT
		DOC "The DeepSort include directory"
		)
		
find_library(DeepSort_LIBRARY
	   NAMES deep_sort_track
	   PATHS ${DeepSort_ROOT_DIR}/lib
	   ENV DEEPSORTROOT
	   ENV LD_LIBRARY_PATH
	   ENV LIBRARY_PATH
	  PATH_SUFFIXES deep_sort)
	  
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(DeepSort DEFAULT_MSG DeepSort_INCLUDE_DIR DeepSort_LIBRARY)
  
if (DeepSort_FOUND)
    set(DeepSort_INCLUDE_DIRS ${DeepSort_INCLUDE_DIR} )
    set(DeepSort_LIBRARIES ${DeepSort_LIBRARY} )
    set(DeepSort_DEFINITIONS )
endif()

# Tell cmake GUIs to ignore the "local" variables.
mark_as_advanced(DeepSort_ROOT_DIR DeepSort_INCLUDE_DIR DeepSort_LIBRARY)
