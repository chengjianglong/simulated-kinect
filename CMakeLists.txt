cmake_minimum_required(VERSION 2.8)

ADD_DEFINITIONS(
	-std=c++11 # Or -std=c++0x
	# Other flags
)

PROJECT(SimuatedKinect)

#---------------------------------------#
# VTK 7.0+
#---------------------------------------#
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#---------------------------------------#
# OGRE library.
#---------------------------------------#

if(EXISTS "/usr/local/lib/OGRE/cmake")

  set(CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
  set(OGRE_SAMPLES_INCLUDEPATH "/usr/local/share/OGRE/samples/Common/include/") # We could just *assume* that developers uses this basepath : /usr/local

elseif(EXISTS "/usr/lib/OGRE/cmake")

  set(CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")
  set(OGRE_SAMPLES_INCLUDEPATH "/usr/share/OGRE/samples/Common/include/") # Otherwise, this one

else ()
  message(SEND_ERROR "Failed to find module path.")
endif(EXISTS "/usr/local/lib/OGRE")
 
 
find_package(OGRE REQUIRED)
 
 
find_package(OIS REQUIRED)
 
if(NOT OIS_FOUND)
	message(SEND_ERROR "Failed to find OIS.")
endif()
 
#----------------------------------------#
# Settings for the projects
#----------------------------------------#

set(sdfpath "/home/chengjiang/Softwares/sdformat/build")
set(IGNITION-MATH_LIBRARIES "/usr/lib/x86_64-linux-gnu/libignition-math2.so.2.3.0")
set(OGREDEPS_INCLUDE_DIR "/home/chengjiang/Softwares/ogredeps/build/ogredeps/include")
set(OGREDEPS_LIBRARY_DIR "/home/chengjiang/Softwares/ogredeps/build/ogredeps/lib")

link_directories(${OGREDEPS_LIBRARY_DIR})

include_directories (
  ${sdfpath}/../include
  ${sdfpath}/include
  ${sdfpath}
  ${Boost_INCLUDE_DIRS}
  /usr/include/ignition/math2
  ${OIS_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_SAMPLES_INCLUDEPATH}
  ${OGRE_Overlay_INCLUDE_DIRS}
)

if (USE_EXTERNAL_URDF)
  link_directories(${URDF_LIBRARY_DIRS})
endif()

if (USE_EXTERNAL_TINYXML)
  link_directories(${tinyxml_LIBRARY_DIRS})
endif()


#-----------------------------------------#
# To-do: Add the application programs.
#-----------------------------------------#

add_executable(VisualizeSDFWorld VisualizeSDFWorld.cpp)
target_link_libraries(VisualizeSDFWorld ${sdfpath}/src/libsdformat.so.4.1.0 ${IGNITION-MATH_LIBRARIES} ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${OGRE_Overlay_LIBRARIES} ${VTK_LIBRARIES} ${OGREDEPS_LIBRARY_DIR}/libFreeImage.a)

add_executable(SimulatedKinect SimulatedKinect.cpp)
target_link_libraries(SimulatedKinect ${sdfpath}/src/libsdformat.so.4.1.0 ${IGNITION-MATH_LIBRARIES} ${OGRE_LIBRARIES} ${OIS_LIBRARIES} ${OGRE_Overlay_LIBRARIES} ${VTK_LIBRARIES} ${OGREDEPS_LIBRARY_DIR}/libFreeImage.a)

