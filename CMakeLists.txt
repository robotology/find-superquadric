# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.0.0)
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

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(IPOPT REQUIRED)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

include_directories(${PROJECT_SOURCE_DIR}/src ${IPOPT_INCLUDE_DIRS})
add_definitions(${IPOPT_DEFINITIONS} -D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} src/nlp.h src/nlp.cpp src/main.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${IPOPT_LIBRARIES})
if(VTK_LIBRARIES)
  target_link_libraries(${PROJECT_NAME} ${VTK_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} vtkHybrid vtkWidgets)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

