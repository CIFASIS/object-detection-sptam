CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT( test-covariance )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake-modules)

## Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

# Ignore annoying Eigen warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-attributes -Wno-misleading-indentation")

## Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# SPTAM source files location
SET(SPTAM_SRC ../../../sptam)
SET(STANDALONE_SRC ../../)

INCLUDE_DIRECTORIES( /usr/include/eigen3 )
INCLUDE_DIRECTORIES( ${SPTAM_SRC} )
INCLUDE_DIRECTORIES( ${STANDALONE_SRC} )

# Find Boost Library
FIND_PACKAGE(Boost COMPONENTS system thread regex REQUIRED)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
# Preserve project Boost required libraries
# This is because of a bug in FindPCL.cmake which overwrites the BOOST
# libs and includes with its own dependencies, instead of merging them.
# fix: http://stackoverflow.com/questions/19720639/handling-multiple-find-package-calls-in-cmake
SET(Boost_PROJECT_LIBRARIES ${Boost_LIBRARIES})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)

# Find PCL library
FIND_PACKAGE(PCL 1.7 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Find Suitesparse library
find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})

# Find G2O Library
find_package(G2O REQUIRED)
# select required components
SET(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D})

SET( SPTAM_FILES ${SPTAM_SRC}/Camera.cpp ${SPTAM_SRC}/CameraPose.cpp ${SPTAM_SRC}/FrustumCulling.cpp ${SPTAM_SRC}/Measurement.cpp ${SPTAM_SRC}/tracker_g2o.cpp ${SPTAM_SRC}/Camera.cpp ${SPTAM_SRC}/g2o_driver.cpp ${SPTAM_SRC}/types_sba_extension.cpp ${SPTAM_SRC}/MapPoint.cpp ${STANDALONE_SRC}/gui/PCLVisualizer.cpp ${SPTAM_SRC}/utils/pose_covariance.cpp )

ADD_EXECUTABLE( test-covariance test-covariance.cpp ${SPTAM_FILES} )
TARGET_LINK_LIBRARIES( test-covariance ${G2O_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YamlCpp_LIBRARIES} ${Boost_LIBRARIES} gdal )
