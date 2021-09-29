cmake_minimum_required(VERSION 2.8.3)
project(dhaiba_ros)
set(${PROJECT_NAME}_VERSION 1.0.0)

## Compile as C++14
add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(URDFDOM urdfdom REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    geometry_msgs
    tf
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

set(PYTHON_EXECUTABLE /usr/bin/python)

set(pybind11_DIR /usr/local/lib/python2.7/dist-packages/pybind11_cmake)
find_package(pybind11 REQUIRED)

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

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
  LIBRARIES
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/${PROJECT_NAME}.so
)

add_executable(urdf_publisher
  src/urdf_publisher.cpp
)

add_dependencies(urdf_publisher
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(urdf_publisher
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${URDFDOM_LIBRARIES}
  ${DhaibaConnectN_LIBRARY}
)

add_executable(test_note_pub
  src/test_note_pub.cpp
)

add_dependencies(test_note_pub
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_note_pub
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${DhaibaConnectN_LIBRARY}
)

add_executable(test_note_sub
  src/test_note_sub.cpp
)

add_dependencies(test_note_sub
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test_note_sub
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${DhaibaConnectN_LIBRARY}
)

add_library(${PROJECT_NAME}
  SHARED src/pybind11_modules.cpp src/dhaiba.cpp)

target_include_directories(${PROJECT_NAME}
  PRIVATE ${PYBIND11_INCLUDE_DIR}
  PRIVATE ${pybind11_INCLUDE_DIR}
  PRIVATE ${PYTHON_INCLUDE_DIRS}
  PRIVATE ${DhaibaConnectN_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${PYTHON_LIBRARIES}
  ${DhaibaConnectN_LIBRARY}
)

set_target_properties(${PROJECT_NAME}
  PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
    LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/
)

#############
## Install ##
#############
install(
  TARGETS
    urdf_publisher test_note_pub test_note_sub
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  PROGRAMS
    scripts/dhaiba_ros_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
