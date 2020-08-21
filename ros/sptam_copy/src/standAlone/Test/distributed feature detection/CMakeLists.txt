CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT(FeatureExtractor)

## Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

## Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Find Boost Library
FIND_PACKAGE(Boost COMPONENTS system filesystem program_options REQUIRED)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})

# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(OpenCV 2 REQUIRED)

message(${OpenCV_LIBS})

# SPTAM source files location
SET(SPTAM_SRC ../../../sptam)


# Compile the program options library
add_library(programOptions ${SPTAM_SRC}/utils/ProgramOptions.cpp)
set(SPTAM_LIBS ${SPTAM_LIBS} programOptions)

ADD_EXECUTABLE( detection detection.cpp ../../../sptam/ImageFeatures.cpp )
TARGET_LINK_LIBRARIES(detection programOptions ${OpenCV_LIBS} ${Boost_LIBRARIES})
