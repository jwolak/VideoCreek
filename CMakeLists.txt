# VideoCreek 1.0
# Author: Janusz Wolak
# Copyrights (C) 2023

#------------------------------------------------------------------------------------------
#                                CMAKE settings
#------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.22.1)

#------------------------------------------------------------------------------------------
#                                VideoCreek dependencies
#------------------------------------------------------------------------------------------
message(STATUS "Searching OpenCV...")
find_package(OpenCV 4.6.0 REQUIRED)

message(STATUS "Searching EquinoxLogger...")
find_package(EquinoxLogger REQUIRED)

#------------------------------------------------------------------------------------------
#                                VideoCreek settings
#------------------------------------------------------------------------------------------
project(VideoCreek VERSION 1.0.0)
message(STATUS "Processing CMakeLists.txt for: " ${PROJECT_NAME} " " ${VERSION})
message(STATUS "CMAKE_SOURCE_DIR:    " ${CMAKE_SOURCE_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++ standard set to: " ${CMAKE_CXX_STANDARD})

#------------------------------------------------------------------------------------------
#                                VideoCreek options
#------------------------------------------------------------------------------------------
option(VIDEO_CREEK_TESTS        "Build tests"        OFF)
if(VIDEO_CREEK_TESTS)
    message(STATUS "Build test enabled")
endif()

#------------------------------------------------------------------------------------------
#                                VideoCreek include
#------------------------------------------------------------------------------------------
set(VIDEO_CREEK_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(VIDEO_CREEK_INCLUDE_INSTANCE_FACTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/VideoCreekInstanceFactory
)

include_directories(${PROJECT_NAME} PRIVATE  ${VIDEO_CREEK_INCLUDE})
include_directories(${PROJECT_NAME} PRIVATE  ${VIDEO_CREEK_INCLUDE_INSTANCE_FACTORY})

#------------------------------------------------------------------------------------------
#                                VideoCreek source directory
#------------------------------------------------------------------------------------------
set(VIDEO_CREEK_SRC_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

#------------------------------------------------------------------------------------------
#                                VideoCreek source
#------------------------------------------------------------------------------------------
set(VIDEO_CREEK_SRC
    ${VIDEO_CREEK_SRC_DIR}/main.cpp
    ${VIDEO_CREEK_SRC_DIR}/CmdArguments.cpp
    ${VIDEO_CREEK_SRC_DIR}/CmdArgumentsParser.cpp
    ${VIDEO_CREEK_SRC_DIR}/VideoCreek.cpp
    ${VIDEO_CREEK_SRC_DIR}/VideoCreekInstanceFactory/ReceiverInstance.cpp
    ${VIDEO_CREEK_SRC_DIR}/VideoCreekInstanceFactory/SenderInstance.cpp
    ${VIDEO_CREEK_SRC_DIR}/VideoCreekInstanceFactory/ReceiverInstanceFactory.cpp
    ${VIDEO_CREEK_SRC_DIR}/VideoCreekInstanceFactory/SenderInstanceFactory.cpp
    ${VIDEO_CREEK_SRC_DIR}/VideoCreekInstanceFactory/VideoCreekInstanceFactory.cpp
    ${VIDEO_CREEK_SRC_DIR}/CameraHandler.cpp
    ${VIDEO_CREEK_SRC_DIR}/CompressionHandler.cpp
    ${VIDEO_CREEK_SRC_DIR}/UdpStreamer.cpp
    ${VIDEO_CREEK_SRC_DIR}/NetworkHandler.cpp
)

#------------------------------------------------------------------------------------------
#                                VideoCreek target
#------------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}.x86 ${VIDEO_CREEK_SRC})

#------------------------------------------------------------------------------------------
#                                VideoCreek target links
#------------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}.x86 ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}.x86 EquinoxLogger)

#------------------------------------------------------------------------------------------
#                                VideoCreek tests
#------------------------------------------------------------------------------------------
if(VIDEO_CREEK_TESTS)
    add_subdirectory(tests)
    message(STATUS "Tests subdirectory added")
endif(VIDEO_CREEK_TESTS)
