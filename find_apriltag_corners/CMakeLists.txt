
cmake_minimum_required(VERSION 2.6)

project(find_apriltag_corners)
include(FindPkgConfig)

pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)

#################
#~ set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#~ set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

#~ if(APPLE)
  #~ include_directories(/opt/local/include) # MacPorts
  #~ link_directories(/opt/local/lib)
  #~ find_library(OPENGL_LIBRARY OpenGL)
#~ else()
  #~ find_library(OPENGL_LIBRARY GL)
  #~ find_library(GLU_LIBRARY GLU)
  #~ set(OPENGL_LIBRARY ${OPENGL_LIBRARY} ${GLU_LIBRARY})
#~ endif()

#~ find_library(GLUT_LIBRARY glut)

#~ include(FindPkgConfig)

#~ pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)
#~ include_directories(${OPENCV_INCLUDE_DIRS})

#~ pkg_check_modules(CAIRO cairo)

#~ if (${CAIRO_FOUND})
#~ add_definitions(-DMZ_HAVE_CAIRO)
#~ endif ()
#################

find_package( CGAL QUIET COMPONENTS  )
 message("CGAL" ${CGAL_FOUND})
if (${CGAL_FOUND})
  include( ${CGAL_USE_FILE} )
  message (STATUS "CGAL " ${CGAL_USE_FILE} )
  add_definitions(-DHAVE_CGAL)
  find_package( Boost REQUIRED )
else()
  message("CGAL not found, can not use new quad detector")
endif()

set(AT_LIBS /home/manuelli/spartan/modules/spartan/mcubeUROP/apriltags-cpp/build/libapriltags.a ${OPENCV_LDFLAGS})

message(STATUS "AT_LIBS" ${OPENCV_LDFLAGS})

include_directories(/home/daniel/software/apriltags-cpp/src)
include_directories(${OPENCV_INCLUDE_DIRS})

project(find_corners)
add_executable(find_corners find_corners.cpp)
target_link_libraries(find_corners ${AT_LIBS}  ${CGAL_LIBRARY})

############################################### BELOW NEEDS TO BE ADJUSTED #################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#include_directories(
# ${catkin_INCLUDE_DIRS}
#  src/ik
#)

## Declare a cpp library
#add_library(image_capture
#   src/${PROJECT_NAME}/image_capture.cpp
# )

## Declare a cpp executable
#add_executable(find_corners_helper find_corners_pythonhelper.cpp)

SET (CMAKE_CXX_FLAGS "-fPIC -g -fno-stack-protector ")
include_directories(
  /usr/include/python2.7
)

ADD_LIBRARY (find_corners_helper_python SHARED
  find_corners_pythonhelper.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
## add_dependencies(ikfast image_capture_generate_messages_cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(ikfast
#  ${catkin_LIBRARIES}
#)

target_link_libraries(find_corners_helper_python ${AT_LIBS} ${CGAL_LIBRARY}
)
