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
  "S-PTAM",
  "S-PTAM+LC",
  "OTHER"
]

colors = [
  (0, 0.4470, 0.7410),
  (0,0.5,0),
  (0.5,0.5,0)
]

class ExperimentData:

  def __init__(self, logfile, grnd_times, grnd_transfs):

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
    
    # filter only the ground truth poses related to the recorded data
    self.computed_timestamps = []
    self.computed_grnd_transformations = []
    
    # WARNING There could be an offset relating ground truth and stam frame numbers.
    for frame_num in self.computed_frames:
      self.computed_timestamps.append( grnd_times[ frame_num-1 ] )
      self.computed_grnd_transformations.append( grnd_transfs[ frame_num-1 ] )
      
    self.computed_grnd_poses = []
    for grnd_transform in self.computed_grnd_transformations:
      R = mh.quaternionToRotationMatrix( grnd_transform[3:] )
      self.computed_grnd_poses.append([R[0][0], R[0][1], R[0][2], grnd_transform[0], R[1][0], R[1][1], R[1][2], grnd_transform[1], R[2][0], R[2][1], R[2][2], grnd_transform[2]])
      
    self.computed_grnd_poses = np.reshape( self.computed_grnd_poses, (len(self.computed_grnd_poses), 3, 4) )

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
  
  parser.add_argument('grnds', help='file containing the ground truth poses and timestamps, corresponds to dumped topic information from level07_20_12_AMCL.bag')
  parser.add_argument('logfiles', help='list of files containing the logged data', nargs='+')

  args = parser.parse_args()
  
  ####################################################################
  # Load Pose data
  ####################################################################

  # load ground truth data  
  #format:
  #%time,field.header.seq,field.header.stamp,field.header.frame_id,field.pose.position.x,field.pose.position.y,field.pose.position.z,field.pose.orientation.x,field.pose.orientation.y,field.pose.orientation.z,field.pose.orientation.w
  grnd_data = np.loadtxt( args.grnds, delimiter=',', usecols=(0,4,5,6,7,8,9,10)) # get only columns for timestamp, position and orientation(quaternion)

  # convert time to seconds
  timestamps_data = (grnd_data[:,0] - grnd_data[0][0]) / 1000000000.0
  ground_truth_poses_data = grnd_data[:,1:]

  # each logfile defines an experiment
  experiments = []

  for logfile, name_override in zip(args.logfiles, label_exp):
    experiments.append( ExperimentData( logfile, timestamps_data, ground_truth_poses_data ) )
    experiments[-1].name = name_override

  pos_grnd = []
  orientation_grnd = []

  for grnd_transform in ground_truth_poses_data:
    pos_grnd.append( grnd_transform[:3] )
    R = mh.quaternionToRotationMatrix( grnd_transform[3:] )
    orientation_grnd.append( R )
    
  print("grnd size", len(ground_truth_poses_data))
    
  # convert data to numpy arrays
  pos_grnd = np.array( pos_grnd )
  orientation_grnd = np.array( orientation_grnd )
    
  ####################################################################
  # General stuffs for plotting
  ####################################################################

  xz_path = []
  xy_path = []
  yx_path = []
  zy_path = []
  lines3D = []

  x_grnd = pos_grnd[:,0]
  y_grnd = pos_grnd[:,1]
  z_grnd = pos_grnd[:,2]

  xz_path.append( (x_grnd, z_grnd, label_grnd, (0, 0, 0)) )
  xy_path.append( (x_grnd, y_grnd, label_grnd, (0, 0, 0)) )
  zy_path.append( (z_grnd, y_grnd, label_grnd, (0, 0, 0)) )
  
  # Flipped axes, as the level7 base_link coordinate system is right handed "X-forward"
  yx_path.append( (-1*y_grnd, x_grnd, label_grnd, (0, 0, 0)) )
  lines3D.append( (x_grnd, y_grnd, z_grnd, label_grnd, (0, 0, 0)) )

  for experiment, color, exp_name in zip(experiments, colors, label_exp):

    x_stam = experiment.predicted_pos[:,0]
    y_stam = experiment.predicted_pos[:,1]
    z_stam = experiment.predicted_pos[:,2]

    xz_path.append( (x_stam, z_stam, experiment.name, color) )
    xy_path.append( (x_stam, y_stam, experiment.name, color) )
    zy_path.append( (z_stam, y_stam, experiment.name, color) )
    
    yx_path.append( (-1*y_stam, x_stam, experiment.name, color) )
    lines3D.append( (x_stam, y_stam, z_stam, experiment.name, color) )

    # add adjusted data, with 0.5 alpha

    x_stam = experiment.adjusted_pos[:,0]
    y_stam = experiment.adjusted_pos[:,1]
    z_stam = experiment.adjusted_pos[:,2]

    xz_path.append( (x_stam, z_stam, experiment.name+"-aligned", color+(0.5,)) )
    zy_path.append( (z_stam, y_stam, experiment.name+"-aligned", color+(0.5,)) )
    
    # There is a small fix as the KFs are in the coordinate system of the camera
    yx_path.append( ((-1*y_stam)+0.155, x_stam, experiment.name+"-aligned", color+(0.5,)) )
    lines3D.append( (x_stam, y_stam, z_stam, experiment.name+"-aligned", color+(0.5,)) )

  ph.plotPaths2D( yx_path )
  #~ plt.savefig('path.png', bbox_inches='tight')

  ph.plotPaths3D( lines3D )
  #~ plt.savefig('path3D.png', bbox_inches='tight')
  
  ####################################################################
  # Plot detected loops
  ####################################################################

  pos_grnd_overtime = []

  # convert data to numpy array  
  np_timestamps_data = np.array(timestamps_data) / 60.
  
  pos_grnd_overtime.append( (x_grnd, y_grnd, np_timestamps_data, (0, 0, 0)) )
  
  loopLines = []  
  for experiment, color in zip(experiments, colors):
    computed_x_grnd = np.array(experiment.computed_grnd_transformations)[:,0]
    computed_y_grnd = np.array(experiment.computed_grnd_transformations)[:,1]
    computed_z_grnd = np.array(experiment.computed_grnd_transformations)[:,2]
    
    np_computed_times = np.array(experiment.computed_timestamps) / 60
    
    for loop in experiment.accepted_loops:
      loopLines.append(([computed_x_grnd[loop[0]], computed_x_grnd[loop[1]]], [computed_y_grnd[loop[0]], computed_y_grnd[loop[1]]], [np_computed_times[loop[0]], np_computed_times[loop[1]]], (1,0,0)))

  ph.plotLoops3D(pos_grnd_overtime, loopLines, time_unit='minutes')
  plt.savefig('loops3D.png', bbox_inches='tight')

  ####################################################################
  # Plot Errors
  ####################################################################

  # convert data to numpy array  
  np_timestamps_data = np.array(timestamps_data)
  
  for experiment in experiments:
    experiment.computeErrors( experiment.computed_grnd_poses )
    
    exp_loops_idx = []
    for loop in experiment.accepted_loops:
      # queryKF, frame number when loop was detected
      query_keyframe = loop[0].astype(int)
      # find nearest computed timestamp with the loop timestamp
      idx = find_nearest(experiment.computed_timestamps, np_timestamps_data[query_keyframe])
      exp_loops_idx.append(idx)
      
    experiment.loops = exp_loops_idx

  comparador.plotManyErrors2([ (experiment.computed_timestamps, experiment.absolute_translation_errors, experiment.loops, experiment.name) for experiment in experiments ], "absolute translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors2([ (experiment.computed_timestamps[1:], experiment.relative_translation_errors, experiment.loops, experiment.name) for experiment in experiments ], "relative translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors2([ (experiment.computed_timestamps, experiment.absolute_rotation_errors, experiment.loops, experiment.name) for experiment in experiments ], "absolute orientation errors", "Time (s)", "Angular deviation (deg)", True)
  comparador.plotManyErrors2([ (experiment.computed_timestamps[1:], experiment.relative_rotation_errors, experiment.loops, experiment.name) for experiment in experiments ], "relative orientation errors", "Time (s)", "Angular deviation (deg)", True)
  
  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()

