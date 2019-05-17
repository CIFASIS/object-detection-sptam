#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
#~ import sg_filter # for low pass smoothing of noisy data

import sptam_parser
import pretty_boxplot
import plotHelpers as ph
import mathHelpers as mh
import comparisonPlotter as comparador

# define labels names for plots
label_grnd = "Ground Truth"
label_stam = "S-PTAM"
label_kf = "S-PTAM Trajectory"

# define colors for plotting
color_grnd = "blue"
color_stam = "green"
color_kf_stam = "red"

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

    detector_type = getConfigValue( lines, "detector:" )
    descriptor_type = getConfigValue( lines, "descriptor:" )
    self.name = detector_type + " / " + descriptor_type

    pose_predicted_data = filterByTaskPoses( lines, 'BASE_LINK_POSE:' )
    self.predicted_poses = np.reshape( pose_predicted_data[:,1:13], (len(pose_predicted_data), 3, 4) )
    self.predicted_ori, self.predicted_pos = mh.decomposeTransformations( self.predicted_poses )
    print("self.pose_predicted_data", len(pose_predicted_data))

    pose_adjusted_data = filterByTaskPoses( lines, 'BASE_LINK_KF:' )
#    self.adjusted_poses = np.reshape( pose_adjusted_data[:,1:13], (len(pose_adjusted_data), 3, 4) )
#    self.adjusted_ori, self.adjusted_pos = mh.decomposeTransformations( self.adjusted_poses )

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
    self.accepted_loops = filterByTask( lines, 'ACCEPTED_LOOP:' )

    self.time_extract_features = filterByTask( lines, 'extraction:' )
    #~ self.time_frustum_filter = filterByTask( lines, 'Frustum:' )
    self.time_find_matches = filterByTask( lines, 'find_matches:' )
    #~ self.time_tracking_refine = filterByTask( lines, 'tracking_refinement:' )
    #~ self.time_create_points = filterByTask( lines, 'CreatePoints:' )
    self.time_add_keyframe = filterByTask( lines, 'AddKeyFrame:' )
    #~ self.time_lock_add_points = filterByTask( lines, 'LockAddPoints:' )
    #~ self.time_lock_frustum = filterByTask( lines, 'LockFrustum:' )
    self.tracking_total_times = filterByTask( lines, 'trackingtotal:' )

    self.visible_points = filterByTask( lines, 'visiblePoints:' )
    self.extracted_points = filterByTask( lines, 'ExtractedPoints:' )
    self.created_new_points = filterByTask( lines, 'created_new_points:' )
    self.matched_features = filterByTask( lines, 'matched_feat_total:' )

    self.measurements_per_point = filterByTask( lines, 'MeasurementCount:' )

  def alignPosesTo(self, pos_ini, ori_ini):
    self.predicted_pos, self.predicted_ori = mh.alignToInitialPose2( self.predicted_pos, self.predicted_ori, pos_ini, ori_ini )
    self.adjusted_pos, self.adjusted_ori = mh.alignToInitialPose2( self.adjusted_pos, self.adjusted_ori, pos_ini, ori_ini )

  def computeErrors(self, ground_truth_poses):

    absolute_error_poses = comparador.__computeErrorPoses__(ground_truth_poses, experiment.predicted_poses)
    relative_error_poses = comparador.__computeRelativeErrorPoses__(ground_truth_poses, experiment.predicted_poses)

    self.absolute_translation_errors = comparador.__computeTranslationError__(absolute_error_poses)
    self.relative_translation_errors = comparador.__computeTranslationError__(relative_error_poses)

    self.absolute_rotation_errors = comparador.__computeRotationError__(absolute_error_poses)
    self.relative_rotation_errors = comparador.__computeRotationError__(relative_error_poses)

    self.RMSE_translation_error = comparador.computeRMSE(self.relative_translation_errors)
    self.RMSE_rotation_error = comparador.computeRMSE(self.relative_rotation_errors)


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

def plotTimes( datas, labels, title ):

  fig = plt.figure()
  ax = fig.add_subplot(111)

  for (data, label) in zip(datas, labels):
    ax.plot( data, label=label )

  ax.set_xlabel("frame number")
  ax.set_ylabel("time (s)")

  ax.grid(True)

  handles, labels = ax.get_legend_handles_labels()
  ax.legend(handles, labels, loc='upper left')

  fig.suptitle( title )

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
  timestamps_data = np.loadtxt( args.times )
  #~ ground_truth_poses_data = np.loadtxt( args.grnds )

  _, ground_truth_poses = sptam_parser.loadGroundTruth('kitti', args.grnds )

  print("times size", len(timestamps_data))

  # filter only the ground truth poses related to the frames of the recorded data
  timestamps = [ timestamps_data[ frame_num ] for frame_num in experiments[0].computed_frames ]
  ground_truth_poses = [ ground_truth_poses[ frame_num ] for frame_num in experiments[0].computed_frames ]
  #~ for frame_num in experiments[0].computed_frames:
    #~ timestamps.append( timestamps_data[ frame_num ] )
    #~ ground_truth_poses.append( ground_truth_poses_data[ frame_num ] )

  #~ ground_truth_poses = np.reshape( ground_truth_poses, (len(ground_truth_poses), 3, 4) )

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

  #~ print('First grnd pose:',pos_grnd[0,:])
  #~ print('First stam pose', pos_stam[0])
  #~ print('Final grnd pose:',pos_grnd[-1,:])
  #~ print('Final stam pose:',pos_stam[-1,:])

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

    x_stam = experiment.predicted_pos[:,0]
    y_stam = experiment.predicted_pos[:,1]
    z_stam = experiment.predicted_pos[:,2]

    xz_path.append( (x_stam, z_stam, experiment.name, color) )
    zy_path.append( (z_stam, y_stam, experiment.name, color) )
    lines3D.append( (x_stam, y_stam, z_stam, experiment.name, color) )

    # add adjusted data, with 0.5 alpha

#    x_stam = experiment.adjusted_pos[:,0]
#    y_stam = experiment.adjusted_pos[:,1]
#    z_stam = experiment.adjusted_pos[:,2]

#    xz_path.append( (x_stam, z_stam, experiment.name+"-aligned", color+(0.5,)) )
#    zy_path.append( (z_stam, y_stam, experiment.name+"-aligned", color+(0.5,)) )
#    lines3D.append( (x_stam, y_stam, z_stam, experiment.name+"-aligned", color+(0.5,)) )

  ph.plotPaths2D( xz_path )
  plt.savefig('path.png', bbox_inches='tight')

  ph.plotPaths2D( zy_path )

  ph.plotPaths3D( lines3D )
  #~ plt.savefig('path3D.png', bbox_inches='tight')

  ####################################################################
  # Compute Errors
  ####################################################################

  # Compute the relative movement between positions
  """
  diff_pos_grnd = pos_grnd[1:,:] - pos_grnd[:-1,:]

  total_dist = 0
  for diff in diff_pos_grnd:
    dist = np.linalg.norm( diff )
    total_dist = total_dist + dist

  for pos_stam, _ in experiments_aligned:

    diff_grnd_stam = pos_grnd - pos_stam
    total_error = 0
    max_error = 0
    for diff in diff_grnd_stam:
      dist = np.linalg.norm(diff)
      total_error = total_error + dist
      if max_error < dist:
        max_error = dist

    print()
    print('Trayectory size:', total_dist)
    print('Final Position Error:', np.linalg.norm(diff_grnd_stam[-1,:]))
    print('Max Error:', max_error)
    print('Average Error:', total_error/len(pos_grnd[:,0]))
  """

  ####################################################################
  # Labels
  ####################################################################

  labels = []
  for experiment in experiments:
    labels.append( experiment.name )

  ####################################################################
  # Plot Tracking times
  ####################################################################
  """
  OJO CON EL COMENTARIO DE ABAJO, NO SE BIEN SI EL CODIGO APLICA EN EL CASO CONCURRENTE
  datas = []
  for experiment in experiments:
    # Cuando se corre de manera secuencial S-PTAM se suma el tiempo  del BA al tracking con la funcion add-keyframe
    # por lo tanto hay que remover dicho tiempo y se hace restando el tiempo de addKeyframe
    # tracking_total_times empieza desde 1s porque cuando inicializa el mapa no se agrega un keyframe
    datas.append( experiment.tracking_total_times[1:,1] - experiment.time_add_keyframe[:,1] )

  #~ plotTimes(datas, labels, "total tracking time")
  pretty_boxplot.boxplot(datas, labels, colors, "total tracking time", "CPU time (s)")
  plt.savefig('tracking_time.png', bbox_inches='tight')
  """
  ####################################################################
  # Plot Extraction times
  ####################################################################

  """
  datas = []
  for experiment in experiments:
    #~ data = sg_filter.savitzky_golay(experiment.time_extract_features[:,1], 101, 3)
    data = experiment.time_extract_features[:,1]
    datas.append( data )

  #~ plotTimes(datas, labels, "extraction + detection time")
  pretty_boxplot.boxplot(datas, labels, colors, "total extraction time", "CPU time (s)")
  plt.savefig('extraction_time.png', bbox_inches='tight')
  """

  ####################################################################
  # Plot non-extraction tracking times
  ####################################################################

  """
  datas = []
  for experiment in experiments:
    #~ data = sg_filter.savitzky_golay(experiment.time_extract_features[:,1], 101, 3)
    data = experiment.tracking_total_times[:,1] - experiment.time_extract_features[:,1]
    datas.append( data )

  #~ plotTimes(datas, labels, "extraction + detection time")
  pretty_boxplot.boxplot(datas, labels, colors, "non-extraction tracking time", "CPU time (s)")
  plt.savefig('non_extraction_time.png', bbox_inches='tight')
  """

  ####################################################################
  # Plot Matching times
  ####################################################################

  """
  datas = []
  for experiment in experiments:
    datas.append( experiment.time_find_matches[:,1] )

  #~ plotTimes(datas, labels, "matching time")
  """

  ####################################################################
  # Plot matched
  ####################################################################

  datas = []
  for experiment in experiments:
    # extracted_points empieza desde 1 porque cuando inicializa el mapa también lo imprime pero no hace matching
    matched_points = experiment.matched_features[:,1]
    datas.append( matched_points )

  plotTimes(datas, labels, "cantidad de matches")

  ####################################################################
  # Plot matched percent
  ####################################################################

  datas = []
  for experiment in experiments:
    # extracted_points empieza desde 1 porque cuando inicializa el mapa también lo imprime pero no hace matching
#    matched_points = experiment.matched_points_stereo[:,1] + experiment.matched_points_only_left[:,1] + experiment.matched_points_only_right[:,1]
    matched_points = experiment.matched_features[:,1]

    datas.append( matched_points / experiment.visible_points[:,1] )

  plotTimes(datas, labels, "porcentaje matcheado")

  ####################################################################
  # Plot map quality
  ####################################################################

#  print()
#  print("final map points:")
#  datas = []
#  for experiment, label in zip(experiments, labels):
#    datas.append( experiment.measurements_per_point[:,1] )
#    print("  ", label, sum(experiment.measurements_per_point[:,1]))

#  #~ plotTimes(datas, labels, "total tracking time")
#  pretty_boxplot.boxplot(datas, labels, colors, "measurements per point", "amount of points")
#  plt.savefig('measurements_per_point.png', bbox_inches='tight')

  ####################################################################
  # Plot Errors
  ####################################################################

  #~ positions_stam = []
  #~ orientations_stam = []
  for experiment in experiments:
    experiment.computeErrors( ground_truth_poses )

    #~ positions_stam.append( (experiment.predicted_pos, experiment.name) )
    #~ orientations_stam.append( (experiment.predicted_ori, experiment.name) )

  frame_nums = range(len(timestamps))

  comparador.plotManyErrors([ (timestamps, experiment.absolute_translation_errors, experiment.name) for experiment in experiments ], "absolute translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors([ (timestamps[1:], experiment.relative_translation_errors, experiment.name) for experiment in experiments ], "relative translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors([ (timestamps, experiment.absolute_rotation_errors, experiment.name) for experiment in experiments ], "absolute orientation errors", "Time (s)", "Angular deviation (deg)", True)
  comparador.plotManyErrors([ (timestamps[1:], experiment.relative_rotation_errors, experiment.name) for experiment in experiments ], "relative orientation errors", "Time (s)", "Angular deviation (deg)", True)

  #~ comparador.plotAbsoluteTranslationError(timestamps, pos_grnd, positions_stam, colors)
  #~ comparador.plotRelativeTranslationError(frame_nums, pos_grnd, positions_stam, colors)

  #~ comparador.plotAbsoluteOrientationError(timestamps, orientation_grnd, orientations_stam, colors)
  #~ comparador.plotRelativeOrientationError(frame_nums, orientation_grnd, orientations_stam, colors)

  print "Translation RMSE: " + str(experiment.RMSE_translation_error)
  print "Rotation RMSE: " + str(experiment.RMSE_rotation_error)

  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()
