cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# for future reference. you can ignore this.
#cmake_policy(SET CMP0012 NEW)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
find_package(OpenNI)

find_package(OpenCV REQUIRED)

find_package(OpenGL)
find_library(freeglut_LIBRARY glut /usr/lib)

include_directories(${OPENGL_INCLUDE_DIR})
find_path( freeglut_INCLUDE_DIR GL/freeglut.h
  /usr/include/GL
  /usr/openwin/share/include
  /usr/openwin/include
  /opt/graphics/OpenGL/include
  /opt/graphics/OpenGL/contrib/libglut
  )
include_directories(${freeglut_INCLUDE_DIR})

rosbuild_add_library (FBO src/FrameBufferObject.cpp)
target_link_libraries (FBO GLEW)

rosbuild_add_library (shaderwrapper src/shader_wrapper.cpp)

rosbuild_add_library (urdf_filter 
  src/urdf_filter.cpp
  src/urdf_renderer.cpp 
  src/renderable.cpp)
target_link_libraries (urdf_filter
  ${OPENGL_LIBRARIES}
  ${freeglut_LIBRARY} 
  ${OpenCV_LIBS}
  FBO
  shaderwrapper
  assimp)
rosbuild_link_boost(urdf_filter signals)

rosbuild_add_executable (urdf_filtered_tracker src/urdf_filtered_tracker.cpp)
target_link_libraries (urdf_filtered_tracker urdf_filter OpenNI)

rosbuild_add_executable (realtime_urdf_filter src/realtime_urdf_filter.cpp)
target_link_libraries (realtime_urdf_filter urdf_filter)

rosbuild_add_library (realtime_urdf_filter_nodelet src/realtime_urdf_filter_nodelet.cpp)
target_link_libraries (realtime_urdf_filter_nodelet urdf_filter)
