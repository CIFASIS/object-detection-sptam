# -*- coding: utf-8 -*-
import numpy as np

from utils import mathHelpers as mh

class KITTI_GT:

  def __init__(self, filename):

    poses = np.loadtxt( filename )

    self.timestamps = None
    self.poses = poses.reshape(len(poses), 3, 4)

  def align(self, sptam_frame_ids, sptam_timestamps, sptam_poses, interpolate=False):

    # Frame id's for the KITTI dataset publish frame ids starting from 1, not 0
    sptam_frame_ids = sptam_frame_ids - 1

    new_poses = mh.alignToInitialPose(sptam_poses, self.poses[ sptam_frame_ids[0] ])

    # S-PTAM may loose frames when tracking, so we only select ground truth poses
    # that correspond to the tracked poses.

    # In KITTI, ground truth poses directly correspond to frames.
    if interpolate:
      self.poses = np.array([ self.poses[ frame_id ] for frame_id in sptam_frame_ids ])

    # Align the first frame of both sequences
    return new_poses

def load( filename ):
  return KITTI_GT( filename )
