#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import matplotlib.pyplot as plt
import pretty_boxplot
import plotHelpers as ph
import mathHelpers as mh
import comparisonPlotter as comparador

colors = [
  (0, 0.4470, 0.7410),
  (0.8500, 0.3250, 0.0980),
  (0.9290, 0.6940, 0.1250),
  (0.4940, 0.1840, 0.5560),
  (0.4660, 0.6740, 0.1880),
  (0.3010, 0.7450, 0.9330),
  (0.6350, 0.0780, 0.1840),
  (0.6290, 0.4940, 0.1250),
  (0.4660, 0.6740, 0.5880)
]


class ExperimentData:

  def __init__(self, logfile):

    f = open(logfile, 'r')
    lines = f.readlines()

    self.name = getConfigValue( lines, "BundleAdjustmentActiveKeyframes:" )

    pose_predicted_data = filterByTaskPoses( lines, 'KITTI_POSE:' )
    self.predicted_ori, self.predicted_pos = mh.decomposeTransformations( pose_predicted_data[:,1:] )
    print("self.pose_predicted_data", len(pose_predicted_data))

    # The frame numbers on which there was a pose prediction.
    # Frames that are not in this list were possibly discarded.
    self.computed_frames = pose_predicted_data[:,0]

def getConfigValue( file, name ):

  for line in file:
    words = line.split()
    if words[1] == name:
      return words[2]

def filterByTaskPoses( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if words[0] == task:
      ret.append( (np.array(words[1:])).astype(float) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )



if __name__ == "__main__":

  # parse program options

  parser = argparse.ArgumentParser()

  parser.add_argument('times', help='file containing a list of timestamps.')
  parser.add_argument('grnds', help='file containing the KITTI ground truth poses.')
  parser.add_argument('logfiles', help='list of files containing the logged data', nargs='+')
  parser.add_argument('--align', dest='align', action='store_true', help='align STAM data to ground truth at the first pose')


  args = parser.parse_args()

  ####################################################################
  # Load Pose data
  ####################################################################

  # each logfile defines an experiment
  experiments = []

  for logfile in args.logfiles:
    experiments.append( ExperimentData( logfile ) )

  # load ground truth data
  timestamps_ = np.loadtxt( args.times )
  grnd_transformations_ = np.loadtxt( args.grnds )

  # filter only the ground truth poses related to the recorded data
  timestamps = []
  grnd_transformations = []
  for frame_num in experiments[0].computed_frames:
    # WARNING There could be an offset relating ground truth and stam frame numbers.
    timestamps.append( timestamps_[ frame_num ] )
    grnd_transformations.append( grnd_transformations_[ frame_num ] )


  ####################################################################
  # Tranform data to ground-truth frame
  ####################################################################
  print("grnd size", len(grnd_transformations))
  orientation_grnd, pos_grnd = mh.decomposeTransformations( grnd_transformations )

  if args.align:
    print("aligning data to ground truth at the first pose")
    for experiment in experiments:
      experiment.alignPosesTo( pos_grnd[ 0 ], orientation_grnd[ 0 ] )

  # convert data to numpy arrays
  pos_grnd = np.array( pos_grnd )


  ####################################################################
  # Plot Errors
  ####################################################################

  positions_stam = []
  orientations_stam = []
  for experiment in experiments:
    positions_stam.append( (experiment.predicted_pos, experiment.name) )
    orientations_stam.append( (experiment.predicted_ori, experiment.name) )

  frame_nums = range(len(timestamps))

  comparador.plotRelativeTranslationError(frame_nums, pos_grnd, positions_stam, colors)
  comparador.plotRelativeOrientationError(frame_nums, orientation_grnd, orientations_stam, colors)


  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()
