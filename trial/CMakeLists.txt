cmake_minimum_required(VERSION 2.8)
project(trial CXX)

add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/..)
# message("CMAKE_CURRENT_LIST_DIR: " ${CMAKE_CURRENT_LIST_DIR})

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

add_definitions(-DEPROSIMA_ALL_DYN_LINK)
add_definitions(-DEPROSIMA_USER_DLL_EXPORT)

include_directories(${DhaibaConnectN_INCLUDE_DIR})

set(SRC_SHAPEBOX1 trial_com.cpp shapebox1.cpp)
add_executable(shapebox1 ${SRC_SHAPEBOX1})
target_link_libraries(shapebox1 ${DhaibaConnectN_LIBRARY})

set(SRC_SHAPEBOX2 trial_com.cpp shapebox2.cpp)
add_executable(shapebox2 ${SRC_SHAPEBOX2})
target_link_libraries(shapebox2 ${DhaibaConnectN_LIBRARY})

set(SRC_SHAPEBOX3 trial_com.cpp shapebox3.cpp)
add_executable(shapebox3 ${SRC_SHAPEBOX3})
target_link_libraries(shapebox3 ${DhaibaConnectN_LIBRARY})

set(SRC_STL1 trial_com.cpp stl1.cpp)
add_executable(stl1 ${SRC_STL1})
target_link_libraries(stl1 ${DhaibaConnectN_LIBRARY})

set(SRC_STL2 trial_com.cpp stl2.cpp)
add_executable(stl2 ${SRC_STL2})
target_link_libraries(stl2 ${DhaibaConnectN_LIBRARY})

