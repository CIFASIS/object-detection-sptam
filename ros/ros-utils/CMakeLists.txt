cmake_minimum_required(VERSION 2.8.3)
project(ros_utils)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)


find_package(catkin REQUIRED COMPONENTS roscpp cmake_modules geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs nav_msgs )
include_directories(${catkin_INCLUDE_DIRS})

ADD_EXECUTABLE(pose_to_path src/pose_to_path.cpp)
TARGET_LINK_LIBRARIES(pose_to_path ${catkin_LIBRARIES})

ADD_EXECUTABLE(odom_to_pose src/odom_to_pose.cpp)
TARGET_LINK_LIBRARIES(odom_to_pose ${catkin_LIBRARIES})

ADD_EXECUTABLE(odom_to_imu src/odom_to_imu.cpp)
TARGET_LINK_LIBRARIES(odom_to_imu ${catkin_LIBRARIES})

ADD_EXECUTABLE(pose_to_odom src/pose_to_odom.cpp)
TARGET_LINK_LIBRARIES(pose_to_odom ${catkin_LIBRARIES})

ADD_EXECUTABLE(poseWithCovariance_to_ellipsoid src/poseWithCovariance_to_ellipsoid.cpp)
TARGET_LINK_LIBRARIES(poseWithCovariance_to_ellipsoid ${catkin_LIBRARIES})
