CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT(bag2o)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../../../cmake-modules)

message(${CMAKE_MODULE_PATH})

# set c++ compilation options ( debugging / warnings / standard )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -std=c++11")

## maximize optimization
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O4")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O4 -g")

# Find Eigen3 Library
find_package(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})

# Find Suitesparse library
find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})

# Find G2O Library
find_package(G2O REQUIRED)
# select required components
SET(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D})
include_directories(${G2O_INCLUDE_DIRS})


SET(SPTAM_SRC ../../../sptam)
INCLUDE_DIRECTORIES(SPTAM_SRC)

# add core libs
FILE(GLOB types_sba_extension_files ${SPTAM_SRC}/types_sba_extension.cpp )

add_library( types_sba_extension ${types_sba_extension_files})

set(TYPES_SBA_EXTENSION_LIB types_sba_extension)


# add core libs
#FILE(GLOB localizationFiles ${SPTAM_SRC}/Localization/*.cpp )
#add_library( Localization ${localizationFiles})

#ADD_EXECUTABLE(bag2o bag2o.cpp)
#TARGET_LINK_LIBRARIES(bag2o  ${G2O_LIBRARY} ${TYPES_SBA_EXTENSION_LIB} ${G2O_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CHOLMOD_LIBRARIES} cxsparse)

ADD_EXECUTABLE(tracking_g2o tracking_g2o.cpp)
TARGET_LINK_LIBRARIES(tracking_g2o  ${G2O_LIBRARY} ${TYPES_SBA_EXTENSION_LIB} ${G2O_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CHOLMOD_LIBRARIES} cxsparse)



