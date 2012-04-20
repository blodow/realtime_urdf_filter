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

cmake_policy(SET CMP0012 NEW)
find_package(OpenCV REQUIRED)
find_package(OpenGL )
FIND_LIBRARY( freeglut_LIBRARY glut /usr/lib)

# disabling this line makes the compiler use the system-wide install of pcl.
# this will conflict with the ros-wide pcl that gets pulled in through the manifest.xml

include_directories(${OPENGL_INCLUDE_DIR})
FIND_PATH( freeglut_INCLUDE_DIR GL/freeglut.h
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

rosbuild_add_executable (realtime_self_filter 
  src/realtime_self_filter.cpp 
  src/urdf_renderer.cpp 
  src/renderable.cpp 
    #src/offscreen_rendering.cpp
  )
target_link_libraries (realtime_self_filter 
  ${OPENGL_LIBRARIES} ${freeglut_LIBRARY} 
  ${OpenCV_LIBS} 
  FBO shaderwrapper)

