cmake_minimum_required (VERSION 2.6)

# nombre del proyecto
project (STAM)

# defininos algunas opciones de compilación
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")

# Encontrar los componentes necesarios de la librería OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# Encontrar los componentes necesarios de la librería PCL
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(/usr/include/vtk-5.8/)

set(STAM_ROOT ../../../stam)
set(STAM_FILES ${STAM_ROOT}/Matching/FrustumCulling.cpp ${STAM_ROOT}/Localization/CameraPose.cpp)
include_directories(${STAM_ROOT})

# agregar un ejecutable al proyecto
add_executable (frustum_test frustum_test.cpp ${STAM_FILES})

# linkear las librerías necesarias al ejecutable
target_link_libraries (frustum_test ${PCL_LIBRARIES} ${OpenCV_LIBS})
