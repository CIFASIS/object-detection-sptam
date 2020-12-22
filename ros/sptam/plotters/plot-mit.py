#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import tf
import sys
import math
import bisect
import argparse
import numpy as np
import matplotlib.pyplot as plt
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

def filterByTask( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if words[0] == task:
      ret.append( (np.array(words[1:])).astype(float) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )

def interpolate(timestamps, values, time):
  """returns the (linear) interpolated value at 'time'

  @param timestamps:  TODO.
  @type timestamps:   list of floats.

  @param values:      TODO.
  @type values:       list of floats, or tuples of floats.

  @param time         TODO.
  @type time:         float.

  @return:            interpolated value at 'time'.
  @rtype:             float.
  """
  
  assert( len(timestamps) == len(values) )
  #assert( timestamps[0] <= time and time < timestamps[-1] )

  if ( time <= timestamps[0] ):
    return values[0];

  if ( timestamps[-1] <= time ):
    return values[-1];

  # bisect.bisect(a, x, lo=0, hi=len(a))¶
  # The returned insertion point i partitions the array
  # a into two halves so that all(val <= x for val in a[:i]) for the
  # left side and all(val > x for val in a[i:]) for the right side.
  bisect_idx = bisect.bisect(timestamps, time)

  # linear interpolation:
  # y = y0 + (y1-y0) (x-x0) / (x1-x0)

  return values[ bisect_idx-1 ] + ( values[ bisect_idx ] - values[ bisect_idx-1 ] ) * ( time - timestamps[ bisect_idx ] ) / ( timestamps[ bisect_idx ] - timestamps[ bisect_idx-1 ] )

def angleDiff( a1, a2 ):

  return math.atan2(math.sin(a2-a1), math.cos(a2-a1))

def interpolateAngle(timestamps, values, time):

  """returns the (linear) interpolated value at 'time'

  @param timestamps:  TODO.
  @type timestamps:   list of floats.

  @param values:      TODO.
  @type values:       list of floats, or tuples of floats.

  @param time         TODO.
  @type time:         float.

  @return:            interpolated value at 'time'.
  @rtype:             float.
  """
  
  assert( len(timestamps) == len(values) )
  #assert( timestamps[0] <= time and time < timestamps[-1] )

  if ( time <= timestamps[0] ):
    return values[0];

  if ( timestamps[-1] <= time ):
    return values[-1];

  # bisect.bisect(a, x, lo=0, hi=len(a))¶
  # The returned insertion point i partitions the array
  # a into two halves so that all(val <= x for val in a[:i]) for the
  # left side and all(val > x for val in a[i:]) for the right side.
  bisect_idx = bisect.bisect(timestamps, time)

  # linear interpolation:
  # y = y0 + alpha * (y1-y0)
  # alpha = (x-x0) / (x1-x0)

  alpha = ( time - timestamps[ bisect_idx-1 ] ) / ( timestamps[ bisect_idx ] - timestamps[ bisect_idx-1 ] )

  diff = angleDiff(values[ bisect_idx-1 ], values[ bisect_idx ])

  return values[ bisect_idx-1 ] + alpha * diff

################################################################################

def parse_gt( filename, at_timestamps=None ):

  data = np.loadtxt( filename, 'float', '#', ',' )

  # convert timestamps to seconds
  timestamps = [float(t)/1000000.0 for t in data[:,0]]

  # load list of (x, y, theta) touples
  xyts = data[:,1:4]

  # Do interpolation if required
  if not at_timestamps==None:

    xs = [ interpolate(timestamps, xyts[:,0], time) for time in at_timestamps ]
    ys = [ interpolate(timestamps, xyts[:,1], time) for time in at_timestamps ]
    # angle interpolation needs to be treated differently because of wrap around issues
    ts = [ interpolateAngle(timestamps, xyts[:,2], time) for time in at_timestamps ]

    xyts = zip(xs, ys, ts)

    timestamps = at_timestamps

  poses = []
  for xyt in xyts:
    T = np.eye(4, 4)
    T[0:3,3] = np.array([xyt[0], xyt[1], 0])
    T[0:3,0:3] = tf.transformations.euler_matrix(0, 0, xyt[2])[0:3,0:3]
    poses.append( T )

  return timestamps, poses

def parse_stam( data ):

  pose_data = filterByTask( data, 'BASE_LINK_POSE:' )

  assert( len(pose_data[0,2:])==12 )

  poses = []
  for pose in pose_data[:,2:]:
    T = np.eye(4, 4)
    T[0:3,0:4] = np.reshape( pose, (3, 4) )
    poses.append( T )

  return pose_data[:,1], poses

def getClosestIndex(timestamps, time):

  if ( time <= timestamps[0] ):
    return 0;

  if ( timestamps[-1] <= time ):
    return len(timestamps)-1;

  # The returned insertion point i partitions the array
  # a into two halves so that all(val <= x for val in a[:i]) for the
  # left side and all(val > x for val in a[i:]) for the right side.
  bisect_idx = bisect.bisect(timestamps, time)

  # the abs() call should not be necesary
  if abs(timestamps[ bisect_idx ]-time) < abs(time-timestamps[ bisect_idx-1 ]):
    return bisect_idx

  return bisect_idx-1;

################################################################################
# MAIN
################################################################################

if __name__ == "__main__":

  # parse program options

  parser = argparse.ArgumentParser()

  parser.add_argument('grnds', help='file containing the MIT ground truth poses.')
  parser.add_argument('filename', help='file containing the logged data')

  args = parser.parse_args()

  ####################################################################
  # Load data
  ####################################################################

  f = open(args.filename, 'r')
  lines = f.readlines()

  st_ts, st_poses = parse_stam( lines )
  gt_ts, gt_poses = parse_gt( args.grnds, st_ts )

  ####################################################################
  # Transform to the same (stam) coord frame
  ####################################################################

  # calculo la transformacion que lleva del frame de ground truth al de stam
  # supongo que las poses no. 100 coinciden, porque si alineo la 0 queda mal
  g_to_s = st_poses[0].dot( np.linalg.inv( gt_poses[0] ) )

  sarasa = []
  for T in gt_poses:
    sarasa.append( g_to_s.dot( T ) )
  gt_poses = np.array( sarasa )

  timestamps = st_ts - st_ts[0]

  ####################################################################
  # PLot paths
  ####################################################################

  gt_Rs, gt_pos = mh.decomposeTransformations44( gt_poses )
  st_Rs, st_pos = mh.decomposeTransformations44( st_poses )

  gt_pos = np.array( gt_pos )
  st_pos = np.array( st_pos )

  #~ for R in gt_Rs:
    #~ if not comparador.isRotationMatrix( R ):
      #~ print("invalid GT rotation matrix")

  #~ for R in st_Rs:
    #~ if not comparador.isRotationMatrix( R ):
      #~ print("invalid ST rotation matrix")

  positions_stam = [ (st_pos, "stam") ]
  orientations_stam = [ (st_Rs, "stam") ]

  comparador.plotAbsoluteTranslationError(timestamps, gt_pos, positions_stam, colors)
  comparador.plotRelativeTranslationError(timestamps, gt_pos, positions_stam, colors)

  comparador.plotAbsoluteOrientationError(timestamps, gt_Rs, orientations_stam, colors)
  comparador.plotRelativeOrientationError(timestamps, gt_Rs, orientations_stam, colors)

  #~ comparador.plotOrientations( [(timestamps, gt_Rs, "ground truth"), (timestamps, st_Rs, "sptam")], colors )

  #~ comparador.plotRelativeOrientations( timestamps, [(gt_Rs, "ground truth")], [colors[0]] )
  #~ comparador.plotRelativeOrientations( timestamps, [(st_Rs, "sptam")], [colors[1]] )
  #~ comparador.plotRelativeOrientations( timestamps, [(gt_Rs, "ground truth"), (st_Rs, "sptam")], colors )

  #~ ph.plotPaths2D([(gt_pos[:,0], gt_pos[:,1], 'Ground Truth'), (st_pos[:,0], st_pos[:,1], 'S-PTAM')], "/home/tfischer/datasets/mit/mit-part3.dat")

  plt.show()
