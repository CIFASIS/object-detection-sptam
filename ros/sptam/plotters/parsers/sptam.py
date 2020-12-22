# -*- coding: utf-8 -*-

import rospy # for Time object
import numpy as np

from utils import sequence_alignment

########################################################################
# private
########################################################################

def __filterLinesWithTag__(lines, tag):

  ret = []

  for line in lines:
    words = line.split()
    if words[0] == tag:
      ret.append( np.array( words[1:] ) )

  if ( len( ret ) < 1 ):
    raise ValueError("ERROR: Requested tag " + tag + " was not measured")

  return np.array( ret )

def __filterPosesWithTag__(lines, tag):

  # A pose line has 49 elements:
  # - 1 for a frame number
  # - 12 for a 3x4 pose matrix [R|t]
  # - 36 for a 6x6 covariance matrix

  data = __filterLinesWithTag__(lines, tag)

  samples = len( data )

  frame_ids = data[:, 0].astype(int)

  poses = np.reshape( data[:, 1:1+12].astype(float), (samples, 3, 4) )

  covariances = np.reshape( data[:, 1+12:].astype(float), (samples, 6, 6) )

  return frame_ids, poses, covariances

def __filterFrameTimestamps__(lines):

  # frame timestamps come as (id, sec, nsec)

  data = __filterLinesWithTag__(lines, "FRAME_TIMESTAMP")

  frame_times = {}
  for frame_id, sec, nsec in data.astype(int):
    frame_times[ frame_id ] = rospy.Time(sec, nsec)

  return frame_times

########################################################################
# public
########################################################################

# create a simple struct to save sptam poses with named attributes
class struct(object) : pass

def loadPosesWithCovariance(filename, tag, start_offset=None, end_offset=None):
  """
  @param filename
    path of the log file to parse.

  @param tag
    pose tag to filter from the log file.

  @param start_offset
    only filter poses after a certain (elapsed) time offset (in seconds).

  @param end_offset
    only filter poses until a certain (remaining) time offset (in seconds).

  @return
    frame_ids: list of N frame id's
    times: list of N rospy.Time objects
    poses: list of N 3x4 [R|t] pose matrices
    covariances: list of N 6x6 covariance matrices
  """

  f = open(filename, 'r')
  lines = f.readlines()

  # this is an array of rospy.Time objects
  frame_times = __filterFrameTimestamps__(lines)

  frame_ids, poses, covariances = __filterPosesWithTag__(lines, tag)

  # Filter times corresponding only to filtered poses
  times = np.array([ frame_times[ frame_id ] for frame_id in frame_ids ])

  if start_offset:

    first_idx = sequence_alignment.getTimestampOffsetStartingIndex(times, start_offset)

    frame_ids = frame_ids[first_idx:]
    times = times[first_idx:]
    poses = poses[first_idx:]
    covariances = covariances[first_idx:]

  if end_offset:

    last_idx = sequence_alignment.getTimestampOffsetEndingLimit(times, end_offset)

    frame_ids = frame_ids[:last_idx]
    times = times[:last_idx]
    poses = poses[:last_idx]
    covariances = covariances[:last_idx]

  ret = struct()
  ret.frame_ids = frame_ids
  ret.times = times
  ret.poses = poses
  ret.covariances = covariances

  return ret
