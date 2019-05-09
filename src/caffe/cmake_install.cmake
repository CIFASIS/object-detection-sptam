# Install script for directory: /home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/src/caffe

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/include/caffe")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/caffe/proto" TYPE FILE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/include/caffe/proto/caffe.pb.h")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcaffe.so.1.0.0-rc4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcaffe.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib/libcaffe.so.1.0.0-rc4"
    "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib/libcaffe.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcaffe.so.1.0.0-rc4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcaffe.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
           NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib/libproto.a")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/python/caffe/proto" TYPE PROGRAM FILES
    "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/include/caffe/proto/caffe_pb2.py"
    "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/__init__.py"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/src/caffe/test/cmake_install.cmake")

endif()

