CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT(sptam)

# set c++ compilation options ( debugging / warnings / standard )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -std=c++11")

# Find Boost Library
FIND_PACKAGE(Boost COMPONENTS system filesystem program_options REQUIRED)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})

# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)

# Load Opencv modules
if(OpenCV_VERSION VERSION_LESS "3.0")
FIND_PACKAGE(OpenCV COMPONENTS core features2d nonfree highgui REQUIRED)
else()
  FIND_PACKAGE(OpenCV COMPONENTS core features2d xfeatures2d highgui REQUIRED)
endif()


ADD_EXECUTABLE( detection detection.cpp ../../../sptam/utils/ProgramOptions.cpp ../../../sptam/ImageFeatures.cpp )
TARGET_LINK_LIBRARIES(detection ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

ADD_EXECUTABLE( example example.cpp ../../../sptam/utils/ProgramOptions.cpp ../../../sptam/ImageFeatures.cpp )
TARGET_LINK_LIBRARIES(example ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
