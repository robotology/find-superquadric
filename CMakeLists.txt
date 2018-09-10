# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.5)
project(find-superquadric)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

if(NOT MSVC)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

find_package(YARP 3.0.0 REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(IPOPT REQUIRED)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
include_directories(${PROJECT_SOURCE_DIR}/src ${IPOPT_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/nlp.h src/nlp.cpp src/main.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${IPOPT_LIBRARIES} ${VTK_LIBRARIES} ctrlLib)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
