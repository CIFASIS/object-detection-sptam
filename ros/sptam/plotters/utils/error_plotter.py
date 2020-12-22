#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bibliography
============

  [1] R. Smith, M. Self, and P. Cheeseman, “Estimating uncertain spatial realtionships in robotics”
  [2] R. Kuemmerle, B. Steder, C. Dornhege, M. Ruhnke, G. Grisetti, C. Stachniss, and A. Kleiner, “On measuring the accuracy of SLAM algorithms”
"""

# plot different metrics to analyze performance against a ground truth

import math
import sg_filter # for low pass smoothing of noisy data
import numpy as np
import matplotlib.pyplot as plt

import mathHelpers as mh

# define colors for plotting
line_color = "green"
SMOOTH_WINDOW_SIZE = None

# Check if matrix is orthogonal
def isRotationMatrix( Q ):
  if not np.allclose( Q.dot( Q.transpose() ), np.eye(3,3) ):
    print( Q.dot( Q.transpose() ) )
    return False
  return True

def __computeAngleFromMatrix__( R ):
  """Compute the magnitude of the rotation defined by the rotation matrix R.

  Conversion equations
  ====================

  From Wikipedia (http://en.wikipedia.org/wiki/Rotation_matrix), the conversion is given by::

      x = Qzy-Qyz
      y = Qxz-Qzx
      z = Qyx-Qxy
      r = hypot(x,hypot(y,z))
      t = Qxx+Qyy+Qzz
      theta = atan2(r,t-1)

  @param R: A 3x3 rotation matrix.
  @type R:  3x3 numpy array
  @return:  the magnitude of the rotation.
  @rtype:   float
  """

  # Axes.
  axis = np.zeros(3, np.float64)
  axis[0] = R[2,1] - R[1,2]
  axis[1] = R[0,2] - R[2,0]
  axis[2] = R[1,0] - R[0,1]

  # Angle.
  r = np.hypot(axis[0], np.hypot(axis[1], axis[2]))
  t = R[0,0] + R[1,1] + R[2,2]
  theta = math.atan2(r, t-1)

  return theta

def __inverseMotionCompositionOperator__(T1, T2):
  """
  @brief Apply the inverse motion composition operator [1] to two poses represented by T1 and T2.
  @param T1, T2 3x4 or 4x4 transformation matrices.
  """

  #~ T2_i = mh.transformationInverse( T2 )
  #~ T1_44 = mh.to44( T1 )
  #~ T2_44_i = mh.to44( T2_i )
  #~ return T2_44_i.dot( T1_44 )

  T1_i = mh.transformationInverse( T1 )
  T1_44_i = mh.to44( T1_i )
  T2_44 = mh.to44( T2 )
  return T1_44_i.dot( T2_44 )

def __computeErrorPoses__(poses1, poses2):
  """
  @brief Given the poses A and B with corresponding transformation matrices T_A and T_B,
  both with respect to a common coordinate frame W. The Absolute pose error between A and B
  can be defined as T_A^-1 * T_B = T_e which is the transform between coordinate frame B and A.

  @param poses1, poses2
    iterables (with same length) of 3x4 or 4x4 transformation matrices.

  @return array of 4x4 transformation matrices.
  """

  assert( len(poses1) == len(poses2) )

  return np.array([ __inverseMotionCompositionOperator__(T1, T2) for T1, T2 in zip(poses1, poses2) ])

def __computeRelativeOrientations__( orientations ):
  """
  @param orientations
    list of 3x3 orientation (rotation) matrices.
  @return
    list of relative angular changes between rotations.
  """
  return [ R1.transpose().dot( R2 ) for R1, R2 in zip(orientations[:-1], orientations[1:]) ]

def __computeRelativePoses__(poses):
  """
  @brief For a sequence of poses, compute the sequence of relative pose transformations.
  For sequential poses T_k and T_{k+1}, T_{rel} = T_k^{-1} * T_{k+1}

  @param poses1, poses2
    iterables (with same length) of 3x4 or 4x4 transformation matrices.

  @return array of 4x4 transformation matrices.
  """
  return __computeErrorPoses__(poses[:-1], poses[1:])

def __computeRelativeErrorPoses__(poses1, poses2):
  """
  @brief 

  @param poses1, poses2
    iterables (with same length) of 3x4 or 4x4 transformation matrices.

  @return array of 4x4 transformation matrices.
  """
  return __computeErrorPoses__(__computeRelativePoses__(poses1), __computeRelativePoses__(poses2))

####################################################################
# Compute orientation errors
####################################################################

def angleFromRotationMatrix(R):
  """
  https://en.wikipedia.org/wiki/Rotation_matrix#Determining_the_angle
  """

  acos = ( np.trace( R ) - 1.0) / 2.0

  # fix porque acos podría ser mayor a 1.0 por errores numéricos
  #~ assert( abs( acos ) <= 1 )
  acos = min(acos, 1.0)
  acos = max(acos, -1.0)

  return math.acos( acos )

def __getRotations__(error_poses):
  """
  @brief Extract 3x3 rotation matrices from an array of 4x4 or 3x4 pose matrices.
  """

  # get 4th column of the error transforms
  #~ assert( all( map( isRotationMatrix, error_poses[:,:3,:3] ) ) )

  return error_poses[:,:3,:3]

def __computeRotationError__( rotation_errors ):
  """
  @brief 
  """

  angles = list(map(angleFromRotationMatrix, rotation_errors))
  return np.degrees( angles )

def __computeRotationErrorFromPoses__(error_poses):
  """
  @brief 
  """

  # get [0:3) columns of the error transforms
  rotation_errors = __getRotations__( error_poses )

  return __computeRotationError__( rotation_errors )

def __computeRotationErrorMatrices__(orientations1, orientations2):

  assert( len(orientations1) == len(orientations2) )

  return np.array([ o1.dot( o2.transpose() ) for o1, o2 in zip(orientations1, orientations2) ])

def __computeAbsoluteRotationError__(orientations1, orientations2):

  assert( len(orientations1) == len(orientations2) )

  return np.array( map(angleFromRotationMatrix, __computeRotationErrorMatrices__(orientations1, orientations2)) )

################################################################################
# Compute translation errors
################################################################################

def __getPositions__(error_poses):
  """
  @brief Extract positions from an array of 4x4 or 3x4 pose matrices.
  """

  # get 4th column of the error transforms
  return error_poses[:,0:3,3]

def __computeTranslationError__(error_poses):
  """
  @brief Compute the norm of the position errors for an array of poses.
  """

  # get 4th column of the error transforms
  position_errors = __getPositions__( error_poses )

  error_norms = np.linalg.norm( position_errors, axis=1 )
  assert( len(position_errors) == len(error_norms) )

  return error_norms

################################################################################
# Another simplified way of computing relative translation errors,
# without multiplying all the pose matrices. Generates less noise.
################################################################################

def __computeAbsoluteTranslationError__(position1, position2):

  return np.linalg.norm(position2 - position1, axis=1)

def __computeAbsoluteTranslationErrors__(positions1, positions2):

  return np.linalg.norm(positions2 - positions1, axis=1)

def __computeRelativeTranslationError__(poses1, poses2):

  positions1 = __getPositions__( poses1 )
  positions2 = __getPositions__( poses2 )

  relative_translations_1 = __computeAbsoluteTranslationError__(positions1[:-1], positions1[1:])
  relative_translations_2 = __computeAbsoluteTranslationError__(positions2[:-1], positions2[1:])

  return np.absolute(relative_translations_2 - relative_translations_1)

################################################################################
# Compute RMSE translation and rotation errors
################################################################################

def square( x ):

  # if it is a numpy array
  try:
    ret = x.dot(x)
  except AttributeError:
    ret = x*x

  return ret

def computeMSE( error_data ):
  """
  Compute mean squared error metric
  """
  sum_squares = 0.

  for error_sample in error_data:
    sum_squares += square(error_sample)

  return sum_squares / len(error_data)

def computeRMSE( error_data ):
  """
  Compute root-mean-square deviation (RMSD) also called root-mean-square error (RMSE).
  """
  return math.sqrt( computeMSE( error_data ) )

################################################################################
# Plot computed errors
################################################################################

def plotManyErrors(plots, title, xlabel="", ylabel="", show_labels=False):
  """
  @param plots list of (times, errors, label) tuples.
  """

  fig = plt.figure()
  ax = fig.add_subplot(111)

  for times, errors, label in plots:
    ax.plot(times, errors, label=label)

  ax.set_xlabel( xlabel )
  ax.set_ylabel( ylabel )

  ax.grid(True)

  fig.suptitle( title )

  if show_labels:
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc='upper left')
    
def plotManyErrors2(plots, title, xlabel="", ylabel="", show_labels=False):
  """
  @param plots list of (times, errors, loops, label) tuples.
  """

  fig = plt.figure()
  ax = fig.add_subplot(111)

  for times, errors, loops, label, color in plots:
    ax.plot(times, errors, label=label, color=color)
    
    for loop in loops:
      plt.plot(times[loop], errors[loop], marker='v',alpha=.7, markersize=8, color='r')        

  ax.set_xlabel( xlabel )
  ax.set_ylabel( ylabel )

  ax.grid(True)

  #fig.suptitle( title )

  if show_labels:
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles, labels, loc='upper left')

def plotErrors(timestamps, errors, title, xlabel="", ylabel=""):

  fig = plt.figure()
  ax = fig.add_subplot(111)

  ax.plot(timestamps, errors)

  ax.set_xlabel( xlabel )
  ax.set_ylabel( ylabel )

  ax.grid(True)

  fig.suptitle( title )

def plotErrorsByAxis(timestamps, errors, title, xlabel=None, ylabels=None):

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  ax_x.plot(timestamps, errors[:,0])
  ax_y.plot(timestamps, errors[:,1])
  ax_z.plot(timestamps, errors[:,2])

  ax_x.axhline(0, c='r')
  ax_y.axhline(0, c='r')
  ax_z.axhline(0, c='r')

  # hide x ticks for all but bottom plot.
  plt.setp([a.get_xticklabels() for a in fig.axes[:-1]], visible=False)

  if ylabels:
    assert( 3 == len(ylabels) )
    ax_x.set_ylabel( ylabels[0] )
    ax_y.set_ylabel( ylabels[1] )
    ax_z.set_ylabel( ylabels[2] )

  if xlabel:
    ax_z.set_xlabel( xlabel )

  ax_x.grid(True)
  ax_y.grid(True)
  ax_z.grid(True)

  fig.suptitle( title )

################################################################################
# Plot tracks
################################################################################

def __plotTracks__(tracks, title, xlabel):
  """
  @param tracks list of (xs, ys, label) tuples. Where positions is 3D positions array.
  """

  fig = plt.figure()
  ax = fig.add_subplot(111)

  for track in tracks:
    ax.plot(timestamps, errors)

  ax.set_xlabel( xlabel )
  ax.set_ylabel( ylabel )

  ax.grid(True)

  fig.suptitle( title )

def plotTracksXY(tracks, title, xlabel):
  """
  @param tracks list of (positions, label) tuples. Where positions is 3D positions array.
  """

  __tracks__ = [ (track[:,0], track[:,1], label) for track, label in tracks ]

  __plotTracks__(__tracks__, title, xlabel)

def plotTracksXZ(tracks, title, xlabel):
  """
  @param tracks list of (positions, label) tuples. Where positions is 3D positions array.
  """

  __tracks__ = [ (track[:,0], track[:,2], label) for track, label in tracks ]

  __plotTracks__(__tracks__, title, xlabel)

def plotTracksByAxis(tracks, title, xlabel):
  """
  @param tracks list of (timestamps, positions, label) tuples. Where positions is 3D positions array.
  """

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  for timestamps, positions, label in tracks:
    ax_x.plot(timestamps, positions[:,0], label=label)
    ax_y.plot(timestamps, positions[:,1], label=label)
    ax_z.plot(timestamps, positions[:,2], label=label)

  # Fine-tune figure; make subplots close to each other and hide x ticks for
  # all but bottom plot.
  #fig.subplots_adjust(hspace=0)
  plt.setp([a.get_xticklabels() for a in fig.axes[:-1]], visible=False)

  ax_x.set_ylabel("x (m)")
  ax_y.set_ylabel("y (m)")
  ax_z.set_ylabel("z (m)")

  ax_z.set_xlabel( xlabel )

  ax_x.grid(True)
  ax_y.grid(True)
  ax_z.grid(True)

  fig.suptitle( title )
