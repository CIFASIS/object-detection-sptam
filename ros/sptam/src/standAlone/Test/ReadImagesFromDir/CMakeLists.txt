cmake_minimum_required (VERSION 2.6)

# nombre del proyecto
project (ReadImagesFromDir)

# defininos algunas opciones de compilación
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -std=c++11")

# Encontrar los componentes necesarios de la librería OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# agregar un ejecutable al proyecto
add_executable(ReadImagesFromDir ReadImagesFromDir.cpp)

# linkear las librerías necesarias al ejecutable
target_link_libraries(ReadImagesFromDir ${OpenCV_LIBS})
