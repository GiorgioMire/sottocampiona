
cmake_minimum_required(VERSION 2.8)
Project (harris CXX)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(CUDA)


#find_path(FREENECT_GRABBER_INCLUDE_DIR openni2pcl.hpp)
#include_directories(${FREENECT_GRABBER_INCLUDE_DIR} )



IF(NOT FREENECT_ROOT)
  IF(EXISTS "/usr/include/libfreenect")
	  SET(FREENECT_ROOT "/usr")
  ELSEIF(EXISTS "/usr/local/include/libfreenect")
	  SET(FREENECT_ROOT "/usr/local")
  ELSE()
    MESSAGE("FREENECT_ROOT not set. Continuing anyway..")
  ENDIF()
ENDIF()

# Include dir
find_path(Freenect_INCLUDE_DIR
  NAMES libfreenect.h
  PATHS ${FREENECT_ROOT}/include/libfreenect 
)

include_directories(${Freenect_INCLUDE_DIR} )

# Finally the library itself
find_library(Freenect_LIBRARY
  NAMES freenect
  PATHS ${FREENECT_ROOT}/lib  /usr/local/lib64/
)

add_executable (harris ../src/harris.cpp )
target_link_libraries( harris  ${PCL_LIBRARIES} ${Freenect_LIBRARY} ${OpenCV_LIBS} ${CUDA_LIBRARIES} )
