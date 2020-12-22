#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
import plotHelpers as ph
import mathHelpers as mh
import comparisonPlotter as comparador
import tf

# define labels names for plots
label_grnd = "Ground Truth"
# experiments names gets override with these
label_exp = [
  "ORB-SLAM2",
  "S-PTAM+LC"
]

colors = [
  (0, 0.4470, 0.7410),
  (0,0.5,0),
]

class SPTAMData:

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

    absolute_error_poses = comparador.__computeErrorPoses__(ground_truth_poses, self.predicted_poses)
    relative_error_poses = comparador.__computeRelativeErrorPoses__(ground_truth_poses, self.predicted_poses)

    self.absolute_translation_errors = comparador.__computeTranslationError__(absolute_error_poses)
    self.relative_translation_errors = comparador.__computeTranslationError__(relative_error_poses)

    self.absolute_rotation_errors = comparador.__computeRotationError__(absolute_error_poses)
    self.relative_rotation_errors = comparador.__computeRotationError__(relative_error_poses)
    
class ORBData:

  def __init__(self, tum_logfile, grnd_times, grnd_transfs):
    
    self.name = 'ORBSLAM'
    
    # load orbslam data  
    data = np.loadtxt(tum_logfile)
    poses_data = data[:,1:] # orientation as quaternions!
    time_data = data[:,0] - data[0][0]
    
    # 4x4 rotation matrix: Transform poses from z_forward to x_forward
    T_xforward = tf.transformations.euler_matrix(-np.pi/2, 0, -np.pi/2, axes='sxyz')
    
    # tf extracted from level7_full dataset
    baselink_to_camera = tf.transformations.quaternion_matrix([0.540677, -0.540247, 0.455780, -0.456143])
    baselink_to_camera[0][3] = 0
    baselink_to_camera[1][3] = -0.15
    baselink_to_camera[2][3] = 1.15
    camera_to_baselink = np.linalg.inv(baselink_to_camera)
    
    xforward_poses = []
  
    for pose in poses_data:
      hmg_pose = tf.transformations.quaternion_matrix(pose[3:]) # 4x4 orientation matrix w/o position
      hmg_pose[0][3] = pose[0] # setting x,y,z position
      hmg_pose[1][3] = pose[1]
      hmg_pose[2][3] = pose[2]
      pose_xforward = baselink_to_camera.dot(hmg_pose) # from his world coordinate system to ours
      pose_xforward = pose_xforward.dot(camera_to_baselink) # from camera to baselink
      xforward_poses.append(pose_xforward[0:3,0:4].flatten()) # 3x4
    
    xforward_poses = np.array(xforward_poses)
    
    self.predicted_poses = np.reshape( xforward_poses, (len(xforward_poses), 3, 4) )
    self.predicted_ori, self.predicted_pos = mh.decomposeTransformations( self.predicted_poses )
    print("self.pose_predicted_data", len(xforward_poses))

    # TODO: Parse adjusted poses!
    #pose_adjusted_data = filterByTaskPoses( lines, 'BASE_LINK_KF:' )
    #self.adjusted_poses = np.reshape( pose_adjusted_data[:,1:], (len(pose_adjusted_data), 3, 4) )
    #self.adjusted_ori, self.adjusted_pos = mh.decomposeTransformations( self.adjusted_poses )
    
    self.accepted_loops = []

    # WARNING ORBSLAM doesnt give to which frame number corresponds the pose predicted
    # so be have to find the nearest ground truth information based on timestamps!    
    # filter only the ground truth poses related to the recorded data
    self.computed_frames = []
    self.computed_timestamps = []
    self.computed_grnd_transformations = []
    
    for i in range(0, len(time_data)):
      frame_num = find_nearest(grnd_times, time_data[i]) # ORB registers the time when the result was obtained! we might need to substract some time for computation!      
      self.computed_frames.append(frame_num)
      self.computed_timestamps.append(grnd_times[frame_num])
      self.computed_grnd_transformations.append( grnd_transfs[ frame_num ] )
      
    self.computed_grnd_poses = []
    for grnd_transform in self.computed_grnd_transformations:
      R = mh.quaternionToRotationMatrix( grnd_transform[3:] )
      self.computed_grnd_poses.append([R[0][0], R[0][1], R[0][2], grnd_transform[0], R[1][0], R[1][1], R[1][2], grnd_transform[1], R[2][0], R[2][1], R[2][2], grnd_transform[2]])
      
    self.computed_grnd_poses = np.reshape( self.computed_grnd_poses, (len(self.computed_grnd_poses), 3, 4) )
    
  def computeErrors(self, ground_truth_poses):
    
    absolute_error_poses = comparador.__computeErrorPoses__(ground_truth_poses, self.predicted_poses)
    relative_error_poses = comparador.__computeRelativeErrorPoses__(ground_truth_poses, self.predicted_poses)

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
      del words[1]
      del words[1]
      ret.append( (np.array(words)).astype(float) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )
  
def find_nearest(array, value):
    return (np.abs(array-value)).argmin()

if __name__ == "__main__":
  # parse program options

  parser = argparse.ArgumentParser()
  
  parser.add_argument('grnds', help='file containing the ground truth poses and timestamps, corresponds to dumped topic information from level07_20_12_AMCL.bag')
  parser.add_argument('orbslam_tum_logfile', help='orbslam logged data on kitti format')
  parser.add_argument('sptam_logfile', help='sptam logged data')

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
  sptam_experiment = SPTAMData(args.sptam_logfile, timestamps_data, ground_truth_poses_data)
  sptam_experiment.name = label_exp[1]
  
  orb_experiment = ORBData(args.orbslam_tum_logfile, timestamps_data, ground_truth_poses_data)
  orb_experiment.name = label_exp[0]

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
  
  ##### ORBSLAM path plotting
  x_stam = orb_experiment.predicted_pos[:,0]
  y_stam = orb_experiment.predicted_pos[:,1]
  z_stam = orb_experiment.predicted_pos[:,2]

  xz_path.append( (x_stam, z_stam, orb_experiment.name, colors[0]) )
  xy_path.append( (x_stam, y_stam, orb_experiment.name, colors[0]) )
  zy_path.append( (z_stam, y_stam, orb_experiment.name, colors[0]) )
  
  yx_path.append( (-1*y_stam, x_stam, orb_experiment.name, colors[0]) )
  lines3D.append( (x_stam, y_stam, z_stam, orb_experiment.name, colors[0]) )

  #### SPTAM path plotting:
  x_stam = sptam_experiment.predicted_pos[:,0]
  y_stam = sptam_experiment.predicted_pos[:,1]
  z_stam = sptam_experiment.predicted_pos[:,2]

  xz_path.append( (x_stam, z_stam, sptam_experiment.name, colors[1]) )
  xy_path.append( (x_stam, y_stam, sptam_experiment.name, colors[1]) )
  zy_path.append( (z_stam, y_stam, sptam_experiment.name, colors[1]) )
  
  yx_path.append( (-1*y_stam, x_stam, sptam_experiment.name, colors[1]) )
  lines3D.append( (x_stam, y_stam, z_stam, sptam_experiment.name, colors[1]) )

  # add adjusted data, with 0.5 alpha

  x_stam = sptam_experiment.adjusted_pos[:,0]
  y_stam = sptam_experiment.adjusted_pos[:,1]
  z_stam = sptam_experiment.adjusted_pos[:,2]

  xz_path.append( (x_stam, z_stam, sptam_experiment.name+"-aligned", colors[1]+(0.5,)) )
  zy_path.append( (z_stam, y_stam, sptam_experiment.name+"-aligned", colors[1]+(0.5,)) )
  
  # There is a small fix as the KFs are in the coordinate system of the camera
  #yx_path.append( ((-1*y_stam)+0.155, x_stam, sptam_experiment.name+"-aligned", colors[1]+(0.5,)) )
  lines3D.append( (x_stam, y_stam, z_stam, sptam_experiment.name+"-aligned", colors[1]+(0.5,)) )

  ph.plotPaths2D( yx_path )
  #~ plt.savefig('path.png', bbox_inches='tight')

  ph.plotPaths3D( lines3D )
  #~ plt.savefig('path3D.png', bbox_inches='tight')

  #####################################################################
  ## Plot Errors
  #####################################################################

  # convert data to numpy array  
  np_timestamps_data = np.array(timestamps_data)
  
  orb_experiment.computeErrors( orb_experiment.computed_grnd_poses )
  sptam_experiment.computeErrors( sptam_experiment.computed_grnd_poses )

  exp_loops_idx = []
  for loop in sptam_experiment.accepted_loops:
    # queryKF, frame number when loop was detected
    query_keyframe = loop[1].astype(int)
    # find nearest computed timestamp with the loop timestamp
    # LOOP TIMES MUST BE FIXED BY THE LOG FIRST TIME REGISTERED!
    idx = find_nearest(sptam_experiment.computed_timestamps, loop[0]-1466193624.145) # FIX TIMESTAMP (FIRST ONE LOGGED)
    exp_loops_idx.append(idx)
    
  sptam_experiment.loops = exp_loops_idx
  orb_experiment.loops = []
  
  orb_experiment.color = colors[0]
  sptam_experiment.color = colors[1]

  comparador.plotManyErrors2([ (experiment.computed_timestamps, experiment.absolute_translation_errors, experiment.loops, experiment.name, experiment.color) for experiment in [orb_experiment, sptam_experiment] ], "absolute translation errors", "Time (s)", "Euclidean distance (m)", True)
  #comparador.plotManyErrors2([ (experiment.computed_timestamps[1:], experiment.relative_translation_errors, experiment.loops, experiment.name, experiment.color) for experiment in [orb_experiment, sptam_experiment] ], "relative translation errors", "Time (s)", "Euclidean distance (m)", True)
  comparador.plotManyErrors2([ (experiment.computed_timestamps, experiment.absolute_rotation_errors, experiment.loops, experiment.name, experiment.color) for experiment in [orb_experiment, sptam_experiment] ], "absolute orientation errors", "Time (s)", "Angular deviation (deg)", True)
  #comparador.plotManyErrors2([ (experiment.computed_timestamps[1:], experiment.relative_rotation_errors, experiment.loops, experiment.name, experiment.color) for experiment in [orb_experiment, sptam_experiment] ], "relative orientation errors", "Time (s)", "Angular deviation (deg)", True)
  
  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()

