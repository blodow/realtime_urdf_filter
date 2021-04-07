cmake_minimum_required(VERSION 3.10)
project(realtime_urdf_filter)

set(OpenGL_GL_PREFERENCE "GLVND")

set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if (NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
    add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
    message(STATUS "Assimp version has unified headers")
  else()
    message(STATUS "Assimp version does not have unified headers")
  endif()
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(ASSIMP_INCLUDE_DIRS )
  set(ASSIMP_LIBRARY_DIRS )
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  urdf
  tf
  resource_retriever
  cv_bridge
  image_transport)

catkin_package(
#  DEPEND
  # CATKIN_DEPENDS
  INCLUDE_DIRS include
  LIBRARIES urdf_filter
  )

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${ASSIMP_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${ASSIMP_LIBRARY_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)
find_package(PkgConfig)
pkg_check_modules(PC_LIBOPENNI REQUIRED libopenni)
find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_library(freeglut_LIBRARY glut /usr/lib)

include_directories(${OPENGL_INCLUDE_DIR} ${PC_LIBOPENNI_INCLUDE_DIRS})

find_path( freeglut_INCLUDE_DIR GL/freeglut.h
  /usr/include/GL
  /usr/openwin/share/include
  /usr/openwin/include
  /opt/graphics/OpenGL/include
  /opt/graphics/OpenGL/contrib/libglut
  )
include_directories(${freeglut_INCLUDE_DIR})

add_library (FBO src/FrameBufferObject.cpp)
target_link_libraries (FBO GLEW)

add_library (shaderwrapper src/shader_wrapper.cpp)

add_library (urdf_filter
  src/urdf_filter.cpp
  src/urdf_renderer.cpp
  src/renderable.cpp)
target_link_libraries (urdf_filter
  ${OPENGL_LIBRARIES}
  ${freeglut_LIBRARY}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  FBO
  shaderwrapper
  ${ASSIMP_LIBRARIES})

add_executable (urdf_filtered_tracker src/urdf_filtered_tracker.cpp)
target_link_libraries (urdf_filtered_tracker urdf_filter ${PC_LIBOPENNI_LIBRARIES} ${catkin_LIBRARIES})

add_executable (realtime_urdf_filter src/realtime_urdf_filter.cpp)
target_link_libraries (realtime_urdf_filter urdf_filter ${catkin_LIBRARIES})

add_library (realtime_urdf_filter_nodelet src/realtime_urdf_filter_nodelet.cpp)
target_link_libraries (realtime_urdf_filter_nodelet urdf_filter ${catkin_LIBRARIES})

install(TARGETS FBO shaderwrapper urdf_filter urdf_filtered_tracker realtime_urdf_filter realtime_urdf_filter_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

install(FILES plugins/nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

foreach(dir launch urdf include/shaders)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(FILES include/SamplesConfig.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY include/GL3
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
