cmake_minimum_required(VERSION 3.10)
project(dhaiba_pybind)
set(${PROJECT_NAME}_VERSION 1.0.0)

find_package(catkin REQUIRED)

set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/..)
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

set(PYBIND11_CPP_STANDARD -std=c++14)

set(DHAIBA_PYBIND "dhaiba_pybind")

add_definitions(-DVERSION_INFO=\"${PROJECT_NAME}_VERSION\")
add_definitions(-DLIBRARY_NAME=${DHAIBA_PYBIND})
add_definitions(-D__MY_TEST__ -g -Wall)

if (NOT "${PROJECT_NAME}_BUILDTOOL_DEPENDS")
  set(${PROJECT_NAME}_BUILDTOOL_DEPENDS "catkin")
endif()

catkin_python_setup()
catkin_package(
  LIBRARIES
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../${PROJECT_NAME}/${DHAIBA_PYBIND}.so
)

add_library(${DHAIBA_PYBIND} SHARED src/pybind11_modules.cpp src/dhaiba.cpp)

target_include_directories(${DHAIBA_PYBIND}
                PRIVATE ${PYBIND11_INCLUDE_DIR}
                PRIVATE ${pybind11_INCLUDE_DIR}
                PRIVATE ${PYTHON_INCLUDE_DIRS}
                PRIVATE ${DhaibaConnectN_INCLUDE_DIR}
                )

target_link_libraries(${DHAIBA_PYBIND}
                                ${PYTHON_LIBRARIES} ${DhaibaConnectN_LIBRARY})
set_target_properties(${DHAIBA_PYBIND}
                                PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                SUFFIX "${PYTHON_MODULE_EXTENSION}")
set_target_properties(${DHAIBA_PYBIND}
                                PROPERTIES CXX_VISIBILITY_PRESET "hidden")

set_target_properties(${DHAIBA_PYBIND} PROPERTIES
                LIBRARY_OUTPUT_DIRECTORY
                ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../${PROJECT_NAME})

install(PROGRAMS
    scripts/dhaiba_connect_topic.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
 
