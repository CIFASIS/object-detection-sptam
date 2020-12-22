#!/usr/bin/env python3
# -*- coding: utf-8 -*-





# Sript to convert tsukuba ground-truth to a 12 vector pose of the form:

  # pose = [ r00 r01 r02 t0
  #          r10 r11 r12 t1
  #          r20 r21 r22 t2 ]



#  NewTsukubaStereoDataset/groundtruth/camera_track.txt -> Ground truth position
#  of the stereo camera (relative to the middle point of the stereo
#  camera's baseline). 1800 poses (one for each frame). Each line contain
#  6 float values: X Y Z A B C. Where (X, Y, Z) is the 3D position of the
#  camera nad (A, B, C) are the Euler angles (in degrees) that represent
#  the camera orientation.


import math
import numpy as np
import argparse
from tf import transformations

# Euler angles are in radians in transformations.py.

if __name__ == "__main__":


  ##############################################################################
  # parse program options
  ##############################################################################

  parser = argparse.ArgumentParser()

  parser.add_argument('grnd', help='path to tsukuba dataset ground truth file (camera_track.txt)')

  args = parser.parse_args()

  ##############################################################################

  # define base_line in meters (tsukuba: base_line = 0.1)
  base_line = 0.1

  # define new ground truth poses (12 elements vector)
  new_ground_truth_poses = []


  # Open ground truth file

  ground_truth_poses_file = open(args.grnd, "r")

  lines = ground_truth_poses_file.readlines()

  for line in lines:
    x,y,z,pitch,yaw,roll= line.split(', ',5)

    # convert values to floats

    x = float( x )
    y = float( y )
    z = float( z )
    yaw = float( yaw )
    pitch = float( pitch )
    roll = float( roll )

    # convert positions to meters (tsukuba is in centemeters)
    x = x / 100.0
    y = y / 100.0
    z = z / 100.0


    # get the orientation matrix from euler angles

    # convert angles to radians
    yaw = math.radians( yaw )
    pitch = math.radians( pitch )
    roll = math.radians( roll )


    ## get position of camera left

    x_axis = np.array([1,0,0,1])
    y_axis = np.array([0,1,0,1])
    z_axis = np.array([0,0,1,1])


    # Create -180 degrees Pitch Rotation Matrix
    pitch_rotation_matrix = transformations.euler_matrix(-np.pi,0,0,'sxyz')

    # x:pitch y:yaw z:roll
    orientation_matrix = transformations.euler_matrix(pitch,yaw,roll,'sxyz')

    # compute camera position
    camera_left_position = np.array([-base_line/2.0, 0, 0, 1])

    # create transformation from camera to middle baseline coordinate system
    T_b_c = np.identity(4)
    T_b_c[0][3] = camera_left_position[0]
    T_b_c[1][3] = camera_left_position[1]
    T_b_c[2][3] = camera_left_position[2]


    # compute tranformation from middle baseline to world coordinate system
    T_w_b = orientation_matrix
    T_w_b[0][3] = x
    T_w_b[1][3] = y
    T_w_b[2][3] = z

    # correct tranformation from middle baseline to world coordinate system
    T_w_bfix = pitch_rotation_matrix.dot( T_w_b )

    T_bfix_c = pitch_rotation_matrix.dot( T_b_c )

    T_w_c = T_w_bfix.dot( T_bfix_c )

    pose = T_w_c[0:3,:]

    # Convert to 12 elements vector format
    pose = pose.ravel().tolist()

    new_ground_truth_poses.append( pose )


  # create new ground truth output file
  new_ground_truth_poses_file = open("tsukuba_ground_truth_poses.txt", "w")
  np.savetxt(new_ground_truth_poses_file,new_ground_truth_poses, fmt="%f")

  # close files
  new_ground_truth_poses_file.close()
  ground_truth_poses_file.close()
