# Install script for directory: /home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/extract_features")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/extract_features")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/finetune_net")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/finetune_net")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/upgrade_net_proto_binary")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_binary")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/train_net")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/train_net")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/test_net")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/test_net")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/net_speed_benchmark")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/net_speed_benchmark")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/compute_image_mean")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/compute_image_mean")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/convert_imageset")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/convert_imageset")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/device_query")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/device_query")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/upgrade_solver_proto_text")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_solver_proto_text")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/upgrade_net_proto_text")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/upgrade_net_proto_text")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe"
         RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/tools/caffe")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe"
         OLD_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu::::::::"
         NEW_RPATH "/home/taihu/Documentos/jcorti_bkp/rcnn/py-faster-rcnn/caffe-fast-rcnn/install/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib:/usr/local/cuda/lib64:/opt/ros/kinetic/lib/x86_64-linux-gnu")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/caffe")
    endif()
  endif()
endif()

