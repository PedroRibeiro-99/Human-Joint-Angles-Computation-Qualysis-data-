cmake_minimum_required(VERSION 3.0.2)
set(PROJECT_NAME qualysis_data)
project(${PROJECT_NAME})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set (SIMULATION 1)
set (RULA_ASSESSMENT 1)
set (COPPELIA_PATH /home/$ENV{USER}/CoppeliaSim_Edu_V4_2_0_Ubuntu18_04)
set (COPPELIA_SCENARIO_PATH ${PROJECT_SOURCE_DIR}/Operator_qualisys.ttt)
set (FILES_PATH ${PROJECT_SOURCE_DIR}/files/tsv_files)
# configuration files
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.hpp"
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  qt_build
  geometry_msgs
  roscpp
  tf
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
find_package(Qt4 REQUIRED)


#set( QT_USE_XML TRUE)
#include(${QT_USE_FILE})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	CATKIN_DEPENDS vrep_common 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
include_directories(${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
##############################################################################
# Qt Environment
##############################################################################
# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui QtXml) # Add the appropriate components to the component list here


##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/${PROJECT_NAME}/*.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)



# -- QCustomPlot
add_definitions(-DQCUSTOMPLOT_USE_LIBRARY)
set(QCUSTOMPLOT_HOME /home/$ENV{USER}/qcustomplot)
include_directories(
    ${QCUSTOMPLOT_HOME}/include
)
link_directories(
    ${QCUSTOMPLOT_HOME}/build
)


# -- QCPDocumentObject
#set(QCPDOCUMENTOBJECT_HOME /home/$ENV{USER}/qcpdocumentobject)
#include_directories(
#    ${QCPDOCUMENTOBJECT_HOME}/include
#)
#link_directories(
#    ${QCPDOCUMENTOBJECT_HOME}/build
#)



## Declare a C++ executable
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

##############################################################################
# Settings
##############################################################################
set(ROBOT 1)

set(HAND 0)



# -- QCustomPlot and QCPDocumentObject
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_libraries(${PROJECT_NAME} qcustomplotd)
else()
    target_link_libraries(${PROJECT_NAME} qcustomplot)
endif()


# configuration files
#configure_file (
  #"${PROJECT_SOURCE_DIR}/include/config.hpp.in"
  #"${PROJECT_SOURCE_DIR}/include/config.hpp"
#)



