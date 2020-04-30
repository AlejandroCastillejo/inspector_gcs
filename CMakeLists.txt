cmake_minimum_required(VERSION 2.8.3)
project(inspector_gcs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation  
  roscpp
  rospy
  roslaunch
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
  geometry_msgs
  nav_msgs
  geographic_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions

# if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
#   find_package(Qt5Widgets REQUIRED)
# else()
#   find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
#   include(${QT_USE_FILE})
# endif()

# Find Qt5 libraries
find_package(Qt5 REQUIRED COMPONENTS 
Core 
Positioning
Gui
Qml
 Widgets
)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(Qt5Positioning CONFIG REQUIRED)
find_package(Qt5Qml CONFIG REQUIRED)
find_package(Qt5Widgets REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

roslint_cpp()

# Configuration to make Qt compiles go smoother
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
include_directories( ${CMAKE_BINARY_DIR} )

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  UavList.msg
  UalState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  gcsCreateMission.srv
  gcsSendMission.srv
  MissionService.srv
  StbyActionService.srv
  PausedStActionService.srv
  StopService.srv
  API_UavList.srv
  API_MissionCommand.srv
  API_MissionFile.srv
  API_GetTopic.srv
  API_GetTopicsList.srv
  uavLink.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs nav_msgs geographic_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  INCLUDE_DIRS ${inspector_gcs_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_cpp std_msgs nav_msgs geometry_msgs
#  DEPENDS system_lib
)


set (inspector_gcs_SRCS
  src/gcs_plugin.cpp
)

set(inspector_gcs_HDRS
  include/inspector_gcs/gcs_plugin.h
)

set(inspector_gcs_UIS
  src/gcs_plugin.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(inspector_gcs_MOCS ${inspector_gcs_HDRS})
  qt5_wrap_ui(inspector_gcs_UIS_H ${inspector_gcs_UIS})
else()
  qt4_wrap_cpp(inspector_gcs_MOCS ${inspector_gcs_HDRS})
  qt4_wrap_ui(inspector_gcs_UIS_H ${inspector_gcs_UIS})
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)
include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Positioning_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Qml_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
)
## Declare a cpp library
add_library(${PROJECT_NAME}
    src/main.cpp src/get_from_json.cpp src/mission_builder.cpp src/camera_calc.cpp src/gcs_services.cpp src/QGCGeo.cc src/UTM.cpp
    ${inspector_gcs_SRCS}
    ${inspector_gcs_MOCS}
    ${inspector_gcs_UIS_H}
    )


## Declare a C++ executable
add_executable(inspector_gcs_node src/main.cpp)

qt5_use_modules(inspector_gcs_node Core Gui Positioning Qml Widgets) 

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(inspector_gcs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(inspector_gcs_node
  ${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} Qt5::Gui)
target_link_libraries(${PROJECT_NAME} Qt5::Positioning)
target_link_libraries(${PROJECT_NAME} Qt5::Qml)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
#   target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
# else()
#   target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
# endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS inspector_gcs inspector_gcs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_inspector_gcs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
