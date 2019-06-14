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
# experiments names gets override with these
label_exp = [
  "SPTAM",
  "SPTAM+LC"
]

colors = [
  (0, 0.4470, 0.7410),
  (0,0.5,0),
]

class ExperimentData:

  def __init__(self, logfile):

    f = open(logfile, 'r')
    lines = f.readlines()

    detector_type = getConfigValue( lines, "detector:" )
    descriptor_type = getConfigValue( lines, "descriptor:" )
    self.name = detector_type + " / " + descriptor_type

    pose_predicted_data = filterByTaskPoses( lines, 'BASE_LINK_POSE:' )
    self.predicted_poses = np.reshape( pose_predicted_data[:,1:], (len(pose_predicted_data), 3, 4) )
    self.predicted_ori, self.predicted_pos = mh.decomposeTransformations( self.predicted_poses )
    print("self.pose_predicted_data", len(pose_predicted_data))

    pose_adjusted_data = filterByTaskPoses( lines, 'BASE_LINK_KF:' )
    self.adjusted_poses = np.reshape( pose_adjusted_data[:,1:], (len(pose_adjusted_data), 3, 4) )
    self.adjusted_ori, self.adjusted_pos = mh.decomposeTransformations( self.adjusted_poses )

    # WARNING predicted frame ID's are computed differently in the ROS version.
    # The ID is taken from the sequence number of the image message.
    # On the other hand, the ID's of the adjusted keyframes are computed
    # by an incremented variable inside the sptam class.
    # This produces an inconsistency in the numeration of associated poses.
    # There could also be an offset relating ground truth and stam frame numbers.

    # The frame numbers on which there was a pose prediction.
    # Frames that are not in this list were possibly discarded.
    self.computed_frames = map(int, pose_predicted_data[:,0])
    
    # Loops accepted, format: [queryKF, loopKF, nMatches, nInliers] KFs ids are in sptam internal numeration
    self.accepted_loops = filterByTaskRaw( lines, 'ACCEPTED_LOOP:' )

  def alignPosesTo(self, pos_ini, ori_ini):
    self.predicted_pos, self.predicted_ori = alignInitialFrame( self.predicted_pos, self.predicted_ori, pos_ini, ori_ini )
    self.adjusted_pos, self.adjusted_ori = alignInitialFrame( self.adjusted_pos, self.adjusted_ori, pos_ini, ori_ini )

  def computeErrors(self, ground_truth_poses):

    absolute_error_poses = comparador.__computeErrorPoses__(ground_truth_poses, experiment.predicted_poses)
    relative_error_poses = comparador.__computeRelativeErrorPoses__(ground_truth_poses, experiment.predicted_poses)

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

def getConfigValue( file, name ):

  for line in file:
    words = line.split()
    if words[1] == name:
      return words[2]

def filterByTask( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if 2<len(words) and words[2] == task:
      ret.append( np.array([float(words[0]), float(words[3])]) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )

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
  
def filterByTaskRaw( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if 2<len(words) and words[2] == task:
      ret.append( (np.array(words[3:])).astype(float) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )
  
def find_nearest(array, value):
    return (np.abs(array-value)).argmin()

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

  for logfile, name_override in zip(args.logfiles, label_exp):
    experiments.append( ExperimentData( logfile ) )
    experiments[-1].name = name_override

  # load ground truth data
  timestamps_data = np.loadtxt( args.times )
  ground_truth_poses_data = np.loadtxt( args.grnds )

  # filter only the ground truth poses related to the frames of the recorded data
  timestamps = []
  ground_truth_poses = []
  for frame_num in experiments[0].computed_frames:
    # Timestamps and ground truth strat from frame 1, since frame 0
    # is supposed to be the origin and the time is supposed to be 0.
    if frame_num == 0:
      timestamps.append( 0.0 )
      # this is a flat identity pose matrix 3x4.
      ground_truth_poses.append( [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0] )
    else:
      # WARNING There could be an offset relating ground truth and stam frame numbers.
      timestamps.append( timestamps_data[ frame_num-1 ] )
      ground_truth_poses.append( ground_truth_poses_data[ frame_num-1 ] )

  ground_truth_poses = np.reshape( ground_truth_poses, (len(ground_truth_poses), 3, 4) )

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

  for experiment, color, exp_name in zip(experiments, colors, label_exp):

    x_stam = experiment.predicted_pos[:,0]
    y_stam = experiment.predicted_pos[:,1]
    z_stam = experiment.predicted_pos[:,2]

    xz_path.append( (x_stam, z_stam, experiment.name, color) )
    zy_path.append( (z_stam, y_stam, experiment.name, color) )
    lines3D.append( (x_stam, y_stam, z_stam, experiment.name, color) )

    # add adjusted data, with 0.5 alpha

    x_stam = experiment.adjusted_pos[:,0]
    y_stam = experiment.adjusted_pos[:,1]
    z_stam = experiment.adjusted_pos[:,2]

    xz_path.append( (x_stam, z_stam, experiment.name+"-aligned", color+(0.5,)) )
    zy_path.append( (z_stam, y_stam, experiment.name+"-aligned", color+(0.5,)) )
    lines3D.append( (x_stam, y_stam, z_stam, experiment.name+"-aligned", color+(0.5,)) )

  ph.plotPaths2D( xz_path )
  #~ plt.savefig('path.png', bbox_inches='tight')

  ph.plotPaths3D( lines3D )
  #~ plt.savefig('path3D.png', bbox_inches='tight')
  
  ####################################################################
  # Plot detected loops
  ####################################################################

  pos_grnd_overtime = []

  # convert computed timestamps to numpy array  
  np_timestamps = np.array( timestamps )
  
  pos_grnd_overtime.append( (x_grnd, z_grnd, np_timestamps, (0, 0, 0)) )
  
  loopLines = []  
  for experiment, color in zip(experiments, colors):
    for loop in experiment.accepted_loops:
      loopLines.append(([x_grnd[loop[0]], x_grnd[loop[1]]], [z_grnd[loop[0]], z_grnd[loop[1]]], [np_timestamps[loop[0]], np_timestamps[loop[1]]], (1,0,0)))

  ph.plotLoops3D(pos_grnd_overtime, loopLines, time_unit='seconds')
  plt.savefig('loops3D.png', bbox_inches='tight')
  
  ####################################################################
  # Plot Errors
  ####################################################################

  # convert data to numpy array  
  np_timestamps_data = np.array(timestamps_data)
  np_ground_truth_poses = np.array(ground_truth_poses)
  
  for experiment in experiments:
    experiment.computeErrors( ground_truth_poses )
    
    exp_loops_idx = []
    for loop in experiment.accepted_loops:
      # queryKF, frame number when loop was detected
      query_keyframe = loop[0].astype(int)
      # find nearest computed timestamp with the loop timestamp
      idx = find_nearest(np_timestamps, np_timestamps_data[query_keyframe])
      exp_loops_idx.append(idx)
      
    experiment.loops = exp_loops_idx

  comparador.plotManyErrors2([ (timestamps, experiment.absolute_translation_errors, experiment.loops, experiment.name) for experiment in experiments ], "absolute translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors2([ (timestamps[1:], experiment.relative_translation_errors, experiment.loops, experiment.name) for experiment in experiments ], "relative translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors2([ (timestamps, experiment.absolute_rotation_errors, experiment.loops, experiment.name) for experiment in experiments ], "absolute orientation errors", "Time (s)", "Angular deviation (deg)", True)
  comparador.plotManyErrors2([ (timestamps[1:], experiment.relative_rotation_errors, experiment.loops, experiment.name) for experiment in experiments ], "relative orientation errors", "Time (s)", "Angular deviation (deg)", True)
  
  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()

