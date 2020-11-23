cmake_minimum_required(VERSION 2.8.3)
project(dhaiba_ros)

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
)

add_executable(${PROJECT_NAME}_bridge
  src/dhaiba_ros_bridge.cpp
)

add_dependencies(${PROJECT_NAME}_bridge
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_bridge
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

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}_bridge test_note_pub test_note_sub
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
