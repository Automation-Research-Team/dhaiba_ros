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

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${URDFDOM_INCLUDE_DIRS}
  /usr/local/include/Dhaiba
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
)

add_executable(${PROJECT_NAME}_bridge
  src/main.cpp
  src/Bridge.cpp
)

add_dependencies(${PROJECT_NAME}_bridge
  ${${PREOJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_bridge
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${URDFDOM_LIBRARIES}
  DhaibaConnectN
)

#############
## Install ##
#############
install(
  TARGETS
    ${PROJECT_NAME}_bridge
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
