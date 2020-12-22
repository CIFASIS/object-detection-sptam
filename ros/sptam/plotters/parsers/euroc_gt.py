# -*- coding: utf-8 -*-

# import 'print' from python3 to make it compatible when executing with python 2
from __future__ import print_function

import numpy as np
import csv
import os
import yaml
import math
import rospy # for Time object
from tf import transformations as transf

from utils import mathHelpers as mh
from utils import sequence_alignment

class EUROC_GT:

  def __init__(self, foldername):
    
    if os.path.isdir( foldername ):
      self.root_path = foldername
    else:
      raise RuntimeError("euroc parser expects path to the ASL groundtruth root folder. ex:/MH_01_easy/mav0")
      
    self.cam0_path = os.path.join(self.root_path, "cam0")
    if not os.path.isdir(self.cam0_path):
      raise RuntimeError("could not find " + self.cam0_path + " directory")
      
    self.groundtruth_path = os.path.join(self.root_path, "state_groundtruth_estimate0")
    if not os.path.isdir(self.groundtruth_path):
      raise RuntimeError("could not find " + self.groundtruth_path + " directory")
      
    self.imu_to_base_link = os.path.join(self.root_path, "base_link.yaml")
    if not os.path.isfile(self.imu_to_base_link):
      raise RuntimeError("could not find " + self.imu_to_base_link)
    
    self.imu_to_cam0 = os.path.join(self.cam0_path, "sensor.yaml")
    if not os.path.isfile(self.imu_to_cam0):
      raise RuntimeError("could not find cam0 extrinsics calibration file " + self.imu_to_cam0)
      
    self.gtruth_csv = os.path.join(self.groundtruth_path, "data.csv")
    if not os.path.isfile(self.gtruth_csv):
      raise RuntimeError("could not find grountruth poses file " + self.gtruth_csv)
    
    # reading groundtruth csv file
    with open(self.gtruth_csv,'r') as dest_f:
      data_iter = csv.reader(dest_f, 
                             delimiter = ',')
      data_iter.next() #discard header (first line)
      data = [data for data in data_iter]
    data_array = np.asarray(data, dtype = float)
    
    # reading imu_to_base_link transformation
    base_link_yaml = yaml.load(file(self.imu_to_base_link,'r'))
    self.imu_to_baselink = np.reshape(np.array(base_link_yaml['T_IBL']['data']), (base_link_yaml['T_IBL']['rows'],base_link_yaml['T_IBL']['cols']))
    
    # reading imu_to_cam0 transformation
    cam0_yaml = yaml.load(file(self.imu_to_cam0,'r'))
    self.imu_to_cam0 = np.reshape(np.array(cam0_yaml['T_BS']['data']), (cam0_yaml['T_BS']['rows'],cam0_yaml['T_BS']['cols']))
    
    # setting timestamps, first 9 numbers (from right to left) represents nanoseconds after that they are seconds
    self.timestamps = np.array([rospy.Time(int(str(tstamp)[:-9]),int(str(tstamp)[-9:])) for tstamp in data_array[:,0].astype(int)])
    
    positions = data_array[:,1:4]    
    # tf library expects (x,y,z,w) and euroc datasets has (w,x,y,z)
    orientation_quaternions = np.concatenate((data_array[:,5:8], np.array([data_array[:,4]]).T), axis=1)
    orientation_matrices = map(lambda q: transf.quaternion_matrix(q)[:3,:3], orientation_quaternions)

    # setting poses matrices
    self.poses = mh.composeTransformations(orientation_matrices, positions)

  def align(self, sptam_frame_ids, sptam_timestamps, sptam_poses, interpolate=False):

    if interpolate:

      # clip sptam sequence to be contained in the ground truth sequence

      start_idx, end_idx = sequence_alignment.getTimestampSubset(self.timestamps, sptam_timestamps)

      if end_idx < len(sptam_timestamps):
        print("WARNING: clipping", len(sptam_timestamps) - end_idx, "measurements from the end of the sptam sequence")
        sptam_frame_ids = sptam_frame_ids[:end_idx]
        sptam_timestamps = sptam_timestamps[:end_idx]
        sptam_poses = sptam_poses[:end_idx]

      if 0 < start_idx:
        print("WARNING: clipping", start_idx, "measurements from the beginning of the sptam sequence")
        sptam_frame_ids = sptam_frame_ids[start_idx:]
        sptam_timestamps = sptam_timestamps[start_idx:]
        sptam_poses = sptam_poses[start_idx:]

      # filter ground truth indexes corresponding to the poses computed by sptam
      ground_truth_idxs = sequence_alignment.find_nearest_timestamps(self.timestamps, sptam_timestamps)
      
      # filtering groundtruth poses and timestamps
      self.poses = np.array([ self.poses[idx] for idx in ground_truth_idxs ])
      self.timestamps = np.array([ self.timestamps[idx] for idx in ground_truth_idxs ])

    # NOTE: for some reason this doesn't align perfectly, there is some error around 0
    self.poses = mh.preMultiplyPoses(self.poses, mh.transformationInverse( self.poses[0] ))
    self.poses = mh.preMultiplyPoses(self.poses, mh.transformationInverse( self.imu_to_baselink ) )
    self.poses = mh.postMultiplyPoses(self.poses, self.imu_to_cam0 )

    # Align the first frame of both sequences
    return sptam_poses

def load( filename ):
  return EUROC_GT( filename )
