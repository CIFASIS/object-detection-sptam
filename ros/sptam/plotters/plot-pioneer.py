#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import matplotlib.pyplot as plt
import plotHelpers as ph
import mathHelpers as mh

################################################################################


def filterByTask( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if words[0] == task:
      ret.append( (np.array(words[1:])).astype(float) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )


################################################################################
# MAIN
################################################################################

if __name__ == "__main__":

  # parse program options

  parser = argparse.ArgumentParser()

  parser.add_argument('filename', help='file containing the logged data')

  args = parser.parse_args()

  ####################################################################
  # Load data
  ####################################################################

  f = open(args.filename, 'r')
  lines = f.readlines()

  sptam_pose_predicted_data = filterByTask( lines, 'KITTI_POSE:' )
  sptam_pose_adjusted_data = filterByTask( lines, 'KITTI_KF:' )

  sptam_transformations = sptam_pose_predicted_data[:,1:]
  sptam_kf_transformations = sptam_pose_adjusted_data[:,1:]

  frame_ini = int( sptam_pose_predicted_data[0][0] )
  frame_end = int( frame_ini + len(sptam_pose_predicted_data) )

  print("processing results for frames", frame_ini, ":", frame_end)

  orientation_sptam, pos_sptam = mh.decomposeTransformations( sptam_transformations )

  orientation_kf_sptam, pos_kf_sptam = mh.decomposeTransformations( sptam_kf_transformations )

  ####################################################################
  # Draw Orientation plots
  ####################################################################


  # Compute Orientation stuff for all poses

  orientations_x_sptam = []
  orientations_y_sptam = []
  orientations_z_sptam = []

  for n in range(0, len(orientation_sptam)):

    # Convert Orientation Matrices to euler angles
    [ori_x_sptam, ori_y_sptam, ori_z_sptam] = mh.eulerAnglesfromRotationMatrix( orientation_sptam[ n ] )
    orientations_x_sptam.append( ori_x_sptam )
    orientations_y_sptam.append( ori_y_sptam )
    orientations_z_sptam.append( ori_z_sptam )

  # Compute Oientation stuff for keyframes poses

  # WARNING: this only work when the Id of the keyframe follows the ground-truth id
  sptam_kf_frames = (sptam_pose_adjusted_data[:,0]).astype(int) # get frame id
  orientations_x_kf_sptam = []
  orientations_y_kf_sptam = []
  orientations_z_kf_sptam = []

  for n in range(0, len(orientation_kf_sptam)):

    # Convert Orientation Matrices to euler angles

    [ori_x_sptam, ori_y_sptam, ori_z_sptam] = mh.eulerAnglesfromRotationMatrix( orientation_kf_sptam[ n ] )
    orientations_x_kf_sptam.append( ori_x_sptam )
    orientations_y_kf_sptam.append( ori_y_sptam )
    orientations_z_kf_sptam.append( ori_z_sptam )

  ####################################################################
  # Draw Trayectory and Traslation plots
  ####################################################################

  pos_sptam = np.array( pos_sptam )
  pos_kf_sptam = np.array( pos_kf_sptam )

  print("First sptam pose", pos_sptam[0])
  print('Final sptam pose:',pos_sptam[-1,:])
  x_sptam = pos_sptam[:,0]
  y_sptam = pos_sptam[:,1]
  z_sptam = pos_sptam[:,2]

  x_kf_sptam = pos_kf_sptam[:,0]
  y_kf_sptam = pos_kf_sptam[:,1]
  z_kf_sptam = pos_kf_sptam[:,2]


  label_sptam = "S-PTAM"
  pointCloudFile = "/home/taihu/Desktop/pasillo_u_reverso.dat"
  ph.plotPaths3D([(x_sptam, y_sptam, z_sptam, label_sptam), (x_kf_sptam, y_kf_sptam, z_kf_sptam, "S-PTAM adjusted")], "System trajectory")
  ph.plotPaths2D([(x_sptam, z_sptam, label_sptam)], pointCloudFile)


  ####################################################################
  # Draw 2D Trayectory
  ####################################################################

  fig = plt.figure()
  ax = fig.add_subplot(111)

  ax.set_axis_off()

  #bg_img = plt.imread('pab1-2do.png')
  #plt.imshow( bg_img )

  ax.plot(x_sptam, y_sptam)
  ax.plot(x_kf_sptam, y_kf_sptam)

  #ax.grid( True )
  plt.gca().set_aspect('equal', adjustable='box')

  plt.savefig("out.pdf", transparent = True)

  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()

