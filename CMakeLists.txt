cmake_minimum_required(VERSION 2.8.3)
project(dhaiba_ros)
set(${PROJECT_NAME}_VERSION 1.0.0)

## Compile as C++14
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)
add_compile_options(-O2)

find_package(PkgConfig REQUIRED)
pkg_check_modules(URDFDOM urdfdom REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    pybind11_catkin
)

set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR})

find_package(DhaibaConnectN REQUIRED)
if (${DhaibaConnectN_FOUND})
  message(STATUS "DhaibaConnectN found")
  message(STATUS "DhaibaConnectN_INCLUDE_DIR: ${DhaibaConnectN_INCLUDE_DIR}")
  message(STATUS "DhaibaConnectN_LIBRARY: ${DhaibaConnectN_LIBRARY}")
else()
  set( DhaibaConnectN_INCLUDE_DIR /usr/local/include/Dhaiba )
  set( DhaibaConnectN_LIBRARY DhaibaConnectN )
  message(STATUS "DhaibaConnectN not found")
  message(STATUS "DhaibaConnectN_INCLUDE_DIR: ${DhaibaConnectN_INCLUDE_DIR}")
  message(STATUS "DhaibaConnectN_LIBRARY: ${DhaibaConnectN_LIBRARY}")
endif()

add_definitions(-DVERSION_INFO=\"${PROJECT_NAME}_VERSION\")
add_definitions(-DLIBRARY_NAME=${PROJECT_NAME})
add_definitions(-D__MY_TEST__ -g -Wall)

if (NOT "${PROJECT_NAME}_BUILDTOOL_DEPENDS")
  set(${PROJECT_NAME}_BUILDTOOL_DEPENDS "catkin")
endif()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${URDFDOM_INCLUDE_DIRS}
  ${DhaibaConnectN_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
  LIBRARIES
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/${PROJECT_NAME}.so
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    pybind11_catkin
)

#
#  urdf_publisher
#
add_executable(
  urdf_publisher
    src/urdf_publisher.cpp
)

add_dependencies(
  urdf_publisher
    ${${PREOJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  urdf_publisher
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${URDFDOM_LIBRARIES}
    ${DhaibaConnectN_LIBRARY}
)

#
#  test_note_pub
#
add_executable(
  test_note_pub
    src/test_note_pub.cpp
    src/dhaiba_note.cpp
)

add_dependencies(
  test_note_pub
    ${${PREOJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  test_note_pub
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${DhaibaConnectN_LIBRARY}
)

#
#  test_note_sub
#
add_executable(
  test_note_sub
    src/test_note_sub.cpp
    src/dhaiba_note.cpp
)

add_dependencies(
  test_note_sub
    ${${PREOJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  test_note_sub
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${DhaibaConnectN_LIBRARY}
)

#
#  pybind11 module
#
pybind_add_module(
  ${PROJECT_NAME} MODULE
    src/dhaiba_note.cpp
)

target_link_libraries(
  ${PROJECT_NAME} PRIVATE
    ${DhaibaConnectN_LIBRARY}
)

#############
## Install ##
#############
install(
  TARGETS
    urdf_publisher test_note_pub test_note_sub ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/dhaiba_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
