
###############################################
# declare program options with default values #
###############################################

SET(SHOW_POINT_CLOUD OFF)
SET(SHOW_TRACKED_FRAMES OFF CACHE BOOL "Enable/Disable OpenCV frame visualization for the tracker.")
SET(SHOW_PROFILING ON CACHE BOOL "Enable/Disable Profiling of each step.")
SET(USE_LOOPCLOSURE OFF CACHE BOOL "Enable/Disable Loopclosure feature.")
SET(PARALLELIZE ON CACHE BOOL "Enable/Disable parallelization using IntelTBB.")

# Set flags
OPTION(SINGLE_THREAD "Run sptam as a single thread application in sequential fashion." OFF)

######################
# Set compiler flags #
######################

## Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Disable annoying Eigen warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

## Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCMAKE_CXX_STANDARD=14")

##########################
# set optimization flags #
##########################

set(OPTIMIZATION_FLAGS "-O4")

# Flags for ARM (particularly tuned for Odroid-XU4)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
  add_definitions(-DEIGEN_DONT_VECTORIZE) # disabling Eigen vectorization allows the following flags (vectorization)
  set(OPTIMIZATION_FLAGS "${OPTIMIZATION_FLAGS} -mcpu=cortex-a15.cortex-a7 -mtune=cortex-a15.cortex-a7 -march=native -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  # NOTE: unsafe-math is required in order for NEON vectorization instructions to be actually generated, since NEON is not IEEE754 compliant. this of course implies possible imprecision
  # in math operations. we tolerate this on ARM since we need it to run fast.
endif()

SET(ENABLE_TRACE OFF CACHE BOOL "Enable options to enable performance monitoring")

if(ENABLE_TRACE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${OPTIMIZATION_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${OPTIMIZATION_FLAGS} -g")

#################
# Check for TBB #
#################

# Find TBB
if (PARALLELIZE)
  find_package(TBB QUIET)
  if (TBB_FOUND)
    message("-- Enabling parallel code")
    add_definitions(-DENABLE_PARALLEL_CODE)
    include_directories(${TBB_INCLUDE_DIRS})
  else()
    message("-- Intel TBB library not found. WARNING: parallelized code will run serially!")
  endif()
endif()

###################################
# Forward CMake flags to compiler #
###################################

if( SHOW_TRACKED_FRAMES )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_TRACKED_FRAMES")
endif()

if( SHOW_PROFILING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PROFILING")
endif()

if( SHOW_POINT_CLOUD )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_POINT_CLOUD")
endif()

if( USE_ODOMETRY )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_ODOMETRY")
endif()

if( USE_LOOPCLOSURE )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LOOPCLOSURE")
endif()

if( DISABLE_LOCALMAPPING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_LOCALMAPPING")
endif()

if( SINGLE_THREAD )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSINGLE_THREAD")
endif()
