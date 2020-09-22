# -*- coding: utf-8 -*-
import numpy as np
import tf.transformations

from utils import mathHelpers as mh

class Level7_GT:

  def __init__(self, filename):

    # Get columns for timestamp (0), position (4,5,6) and orientation quaternion (7,8,9,10)
    data = np.loadtxt(filename, delimiter=',', usecols=(0,4,5,6,7,8,9,10))

    # parse timestamps
    timestamps = data[:,0]
    # convert time to seconds and offset to 0
    self.timestamps = (timestamps - timestamps[0]) / 1000000000.0

    positions = data[:,1:4]

    orientation_quaternions = data[:,4:8]
    orientation_matrices = map(lambda q: transformations.quaternion_matrix(q)[:3,:3], orientation_quaternions)

    self.poses = mh.composeTransformations(orientation_matrices, positions)

  def align(self, sptam_frame_ids, sptam_timestamps, sptam_poses, interpolate=False):

    raise RuntimeError("not implemented... Gaston?")

def load( filename ):
  return Level7_GT( filename )
