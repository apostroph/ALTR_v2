# Copyright: 2013 Robotcub Consortium
# Author: Konstantinos Theofilis
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8.7)
set(PROJECTNAME predictionModule)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

add_definitions(-std=gnu++0x)

file(GLOB folder_source
    *.cpp
    ./src/*.cpp
)

file(GLOB folder_header
    *.h
    ./include/iCub/*.h
    ./include/iCub/*.hpp
)

include_directories(${YARP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

add_executable(${PROJECTNAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBS})

# to install data files:
include(YarpInstallationHelpers)
set(conf build/config.ini)
# alternatively, you can glob all the files with:
# file(GLOB conf conf/*.ini)

yarp_install(FILES ${conf} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/prediction)
