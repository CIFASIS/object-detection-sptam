#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import plotHelpers as ph
import mathHelpers as mh
import comparisonPlotter as comparador

# define labels names for plots
label_grnd = "Ground Truth"
label_slam = "S-PTAM"
label_slam_other = "ORB-SLAM"

label_exp = [
label_slam,
label_slam_other
]

# define colors for plotting
color_grnd = "black"
color_slam = "green"
color_slam_other = "red"

colors = [
  (0, 0.4470, 0.7410),
  (0.5,0.5,0)
]


class ExperimentData:

  def __init__(self, poses):

    self.name ="method"

    self.poses = np.reshape( poses, (len( poses ), 3, 4) )
    self.orientation, self.position = mh.decomposeTransformations( self.poses )
    print(self.name, len( poses ))

  def alignPosesTo(self, pos_ini, ori_ini):
    self.predicted_pos, self.orientation = alignInitialFrame( self.predicted_pos, self.orientation, pos_ini, ori_ini )

  def computeErrors(self, ground_truth_poses):

    absolute_error_poses = comparador.__computeErrorPoses__(ground_truth_poses, experiment.poses)
    relative_error_poses = comparador.__computeRelativeErrorPoses__(ground_truth_poses, experiment.poses)

    self.absolute_translation_errors = comparador.__computeTranslationError__(absolute_error_poses)
    self.relative_translation_errors = comparador.__computeTranslationError__(relative_error_poses)

    self.absolute_rotation_errors = comparador.__computeRotationError__(absolute_error_poses)
    self.relative_rotation_errors = comparador.__computeRotationError__(relative_error_poses)

def alignInitialFrame( poss, oris, pos_to, ori_to ):

  poss_new = []
  for pos in poss:
    pos_new = ori_to.dot( pos ) + pos_to[ 0 ]
    poss_new.append( pos_new )

  oris_new = []
  for ori in oris:
    ori_new = ori_to.dot( ori )
    oris_new.append( ori_new )

  # convert data to numpy arrays
  return np.array( poss_new ), np.array( oris_new )

if __name__ == "__main__":

  # parse program options

  parser = argparse.ArgumentParser()

  parser.add_argument('times', help='file containing a list of timestamps.')
  parser.add_argument('grnds', help='file containing the KITTI ground truth poses.')
  parser.add_argument('slam_files', help='list of files containing the poses estimated by SLAM methods', nargs='+')
  parser.add_argument('--align', dest='align', action='store_true', help='align SLAM data to ground truth at the first pose')

  args = parser.parse_args()

  ####################################################################
  # Load Pose data
  ####################################################################

  # load ground truth data
  timestamps = np.loadtxt( args.times )
  ground_truth_poses = np.loadtxt( args.grnds )
  ground_truth_poses = np.reshape( ground_truth_poses, (len( ground_truth_poses ), 3, 4) )


  # each slam_file defines an experiment
  experiments = []

  for slam_file in args.slam_files:
    slam_methods_poses = np.loadtxt( slam_file )
    experiments.append( ExperimentData( slam_methods_poses ) )

    # check that all grnd and all slam methods have the same number of poses
    assert( len (ground_truth_poses) == len ( slam_methods_poses ))

  ####################################################################
  # Tranform data to ground-truth frame
  ####################################################################
  print("grnd size", len(ground_truth_poses))
  orientation_grnd, pos_grnd = mh.decomposeTransformations( ground_truth_poses )

  if args.align:
    print("aligning data to ground truth at the first pose")
    for experiment in experiments:
      experiment.alignPosesTo( pos_grnd[ 0 ], orientation_grnd[ 0 ] )

  # convert data to numpy arrays
  pos_grnd = np.array( pos_grnd )

  ####################################################################
  # Labels
  ####################################################################


  for experiment,label in zip(experiments, label_exp):
    experiment.name = label


  ####################################################################
  # General stuffs for plotting
  ####################################################################

  xz_path = []
  zy_path = []
  lines3D = []

  x_grnd = pos_grnd[:,0]
  y_grnd = pos_grnd[:,1]
  z_grnd = pos_grnd[:,2]

  xz_path.append( (x_grnd, z_grnd, label_grnd, (0, 0, 0)) )
  zy_path.append( (z_grnd, y_grnd, label_grnd, (0, 0, 0)) )
  lines3D.append( (x_grnd, y_grnd, z_grnd, label_grnd, (0, 0, 0)) )

  for experiment, color in zip(experiments, colors):

    x_slam = experiment.position[:,0]
    y_slam = experiment.position[:,1]
    z_slam = experiment.position[:,2]

    xz_path.append( (x_slam, z_slam, experiment.name, color) )
    zy_path.append( (z_slam, y_slam, experiment.name, color) )
    lines3D.append( (x_slam, y_slam, z_slam, experiment.name, color) )

  ph.plotPaths2D( xz_path )
  plt.savefig('path.png', bbox_inches='tight')

  ph.plotPaths2D( zy_path )

  ph.plotPaths3D( lines3D )

  ####################################################################
  # Plot Errors
  ####################################################################

  for experiment in experiments:
    experiment.computeErrors( ground_truth_poses )

  frame_nums = range(len(timestamps))

  comparador.plotManyErrors([ (timestamps, experiment.absolute_translation_errors, experiment.name) for experiment in experiments ], "absolute translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors([ (timestamps[1:], experiment.relative_translation_errors, experiment.name) for experiment in experiments ], "relative translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors([ (timestamps, experiment.absolute_rotation_errors, experiment.name) for experiment in experiments ], "absolute orientation errors", "Time (s)", "Angular deviation (deg)", True)
  comparador.plotManyErrors([ (timestamps[1:], experiment.relative_rotation_errors, experiment.name) for experiment in experiments ], "relative orientation errors", "Time (s)", "Angular deviation (deg)", True)

  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()
