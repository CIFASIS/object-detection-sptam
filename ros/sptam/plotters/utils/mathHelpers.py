# -*- coding: utf-8 -*-

import math
import numpy as np

################################################################################
# ANGLE MATH
################################################################################

def quaternionToRotationMatrix( q ):

  # algorithm expects q = [ qw, qx, qy, qz ] and we use
  # q = [ qw, qx, qy, qz ], so we have to set it differently.
  q = np.array([ q[3], q[0], q[1], q[2] ])

  n = np.dot(q, q)

  if n < np.finfo(np.float64).eps:
      return np.identity(3)

  q *= math.sqrt(2.0 / n)
  q = np.outer(q, q)
  return np.array([
    [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0]],
    [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0]],
    [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2]]
  ])

def eulerAnglesfromRotationMatrix( R ):

  r00 = R[0][0]
  r10 = R[1][0]
  r20 = R[2][0]
  r21 = R[2][1]
  r22 = R[2][2]

  theta_x = math.atan2(r21, r22)
  theta_y = math.atan2(-r20, math.sqrt( r21 * r21 + r22 * r22 ))
  theta_z = math.atan2(r10, r00)

  # convert to degrees
  theta_x = math.degrees( theta_x )
  theta_y = math.degrees( theta_y )
  theta_z = math.degrees( theta_z )

  angles = [theta_x, theta_y , theta_z]

  return angles

def R_axis_angle(angle, axis):
    """Generate the rotation matrix from the axis-angle notation.

    Conversion equations
    ====================

    From Wikipedia (http://en.wikipedia.org/wiki/Rotation_matrix), the conversion is given by::

        c = cos(angle); s = sin(angle); C = 1-c
        xs = x*s;   ys = y*s;   zs = z*s
        xC = x*C;   yC = y*C;   zC = z*C
        xyC = x*yC; yzC = y*zC; zxC = z*xC
        [ x*xC+c   xyC-zs   zxC+ys ]
        [ xyC+zs   y*yC+c   yzC-xs ]
        [ zxC-ys   yzC+xs   z*zC+c ]


    @param axis:    The 3D rotation axis.
    @type axis:     numpy array, len 3
    @param angle:   The rotation angle.
    @type angle:    float
    """

    # Trig factors.
    ca = math.cos(angle)
    sa = math.sin(angle)
    C = 1 - ca

    # Depack the axis.
    x, y, z = axis

    # Multiplications (to remove duplicate calculations).
    xs = x*sa
    ys = y*sa
    zs = z*sa
    xC = x*C
    yC = y*C
    zC = z*C
    xyC = x*yC
    yzC = y*zC
    zxC = z*xC

    # Create the rotation matrix.

    matrix = np.zeros((3,3))

    matrix[0, 0] = x*xC + ca
    matrix[0, 1] = xyC - zs
    matrix[0, 2] = zxC + ys
    matrix[1, 0] = xyC + zs
    matrix[1, 1] = y*yC + ca
    matrix[1, 2] = yzC - xs
    matrix[2, 0] = zxC - ys
    matrix[2, 1] = yzC + xs
    matrix[2, 2] = z*zC + ca

    return matrix

################################################################################
# TRANSFORMATION MATH
################################################################################

def to44( T ):
  """
  @brief compute a transformation in 4x4 matrix form.
  @param T 3x4 or 4x4 transformation matrix.
  @return 4x4 transformation matrix.
  """
  T44 = np.eye(4, 4)
  T44[:3,:4] = T[:3,:4]
  return T44

def eyeFromShape( T ):
  """
  @brief create an identity matrix that mimics the shape of another 2D matrix.
  @param T 2D matrix whose shape will be used.
  @return identity matrix with the same shape as T.
  """
  return np.eye( T.shape[0], T.shape[1] )

def composeTransformation( R, t, T ):
  """
  @brief create a transfomation matrix from a rotation matrix and translation vector.
  @param R rotation matrix.
  @param t translation vector.
  @param T matrix on which the transformation will be written. Probably np.eye(3,4) or np.eye(4,4).
  @return the composed transformation.
  """

  T[0:3, 0:3] = R
  T[0:3, 3] = t

  return T

def composeTransformations( Rs, ts ):

  assert( len(Rs) == len(ts) )

  transformations = []

  for R, t in zip(Rs, ts):

    T = np.eye(4, 4)
    T = composeTransformation(R, t, T)

    transformations.append( T )

  return transformations

def decomposeTransformation( T ):
  """
  @brief decompose a 3x4 or 4x4 transformation matrix into R and t matrices.
  @param T 3x4 or 4x4 transformation matrix.
  @return rotation matrix R, translation vector t.
  """

  return T[0:3, 0:3], T[0:3, 3]

def decomposeTransformations( transformations ):
  """
  @brief decompose array of transformations into R and t arrays.
  @param transformations array of 3x4 or 4x4 matrices.
  @return array of 3x3 rotation matrices, array of 3D translation vectors
  """

  Rs = []
  ts = []

  for transform in transformations:

    R, t = decomposeTransformation( transform )

    Rs.append( R )
    ts.append( t )

  return np.array( Rs ), np.array( ts )

def transformationInverse( T ):
  """
  @brief compute the inverse of a transformation matrix.
  @param T 3x4 or 4x4 transformation matrix.
  @return the matrix inverse of T.
  """

  R, t = decomposeTransformation( T )

  Ri = R.transpose()
  ti = -Ri.dot( t )

  Ti = eyeFromShape( T )
  return composeTransformation(Ri, ti, Ti)

def getPositionsFromTransformations( Rs, ts ):

  ps = []

  for R, t in zip(Rs, ts):

    pos = -(R.transpose()).dot( t )

    ps.append( pos )

  return ps

def preMultiplyPoses(poses, T):
  """
  @brief pre-multiply an array of poses by a transformation matrix T (T*P),
    independently of the transformation represantation. Both poses and 'T'
    are allowed to be 3x4 or 4x4 matrices.
  """

  T44 = to44( T )

  new_poses = []
  for pose in poses:
    new_pose = T44.dot( to44( pose ) )
    new_poses.append( new_pose[:pose.shape[0],:pose.shape[1]] )

  return np.array( new_poses )

def postMultiplyPoses(poses, T):
  """
  @brief post-multiply an array of poses by a transformation matrix T (P*T),
    independently of the transformation represantation. Both poses and 'T'
    are allowed to be 3x4 or 4x4 matrices.
  """

  T44 = to44( T )

  new_poses = []
  for pose in poses:
    new_pose = to44( pose ).dot( T44 )
    new_poses.append( new_pose[:pose.shape[0],:pose.shape[1]] )

  return np.array( new_poses )

def alignToInitialPose( poses, initial_pose ):
  """
  @brief Align a path to some intial pose (given in the same reference frame W).
  @param poses array of 3x4 or 4x4 matrices.
  @param initial_pose 3x4 or 4x4 matrix.
  @return array of transformed poses with the same shape as the input poses.
  """

  path_to_world = to44( poses[0] )
  target_to_world = to44( initial_pose )

  # compute the desired transformation
  T = target_to_world.dot( transformationInverse( path_to_world ) )

  return preMultiplyPoses(poses, T)

def alignToInitialPose2( positions, orientations, pos_ini, ori_ini ):
  """
  @brief Align a path to some intial pose (given in the same reference frame W).
  @param positions array of 3D vectors.
  @param orientations array of 3x3 matrices.
  @param pos_ini 3D vector.
  @param ori_ini 3x3 matrix.
  @return array of transformed positions, array of transformed orientations.
  """

  positions_new = []
  for pos in positions:
    pos_new = ori_ini.dot( pos ) + pos_ini
    positions_new.append( pos_new )

  orientations_new = []
  for ori in orientations:
    ori_new = ori_ini.dot( ori )
    orientations_new.append( ori_new )

  return positions_new, orientations_new
