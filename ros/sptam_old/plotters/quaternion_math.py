# -*- coding: utf-8 -*-

import math
import numpy as np

EPSILON = 2.22e-16

# q: 4x1 vector (quaternion)
# ret: 4x1 vector (quaternion)
def qNormalized(q):

  return q * ( 1.0 / np.linalg.norm( q ) )

# v: 3x1 vector
# ret: 4x1 vector (quaternion)
def anglesToQuaternion(v):

  norm = np.linalg.norm( v )

  quat = np.array([1.0, 0.0, 0.0, 0.0])

  if EPSILON <= norm:

    normDiv2 = norm / 2.0
    sinNormDiv2 = math.sin( normDiv2 )

    quat[0] = math.cos( normDiv2 )

    quat[1] = sinNormDiv2 * v[0] / norm
    quat[2] = sinNormDiv2 * v[1] / norm
    quat[3] = sinNormDiv2 * v[2] / norm

  return quat

# q: 4x1 vector (quaternion)
# ret: 3x1 vector
def quaternionToAngles(q):

  q0 = q[0]
  q1 = q[1]
  q2 = q[2]
  q3 = q[3]

  angles = np.array([0.0, 0.0, 0.0])

  angles[0] = math.atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2))
  angles[1] = math.asin(2.0*(q0*q2 - q3*q1))
  angles[2] = math.atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))

  return angles

# Hamilton product for Quaternions
# q1: 4x1 vector (quaternion)
# q2: 4x1 vector (quaternion)
# ret: 4x1 vector (quaternion)
def qMultiply(q1, q2):

  w1 = q1[0]
  x1 = q1[1]
  y1 = q1[2]
  z1 = q1[3]

  w2 = q2[0]
  x2 = q2[1]
  y2 = q2[2]
  z2 = q2[3]

  q_res = np.array([0.0, 0.0, 0.0, 0.0])

  q_res[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2 # w component
  q_res[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2 # x component
  q_res[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2 # y component
  q_res[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2 # z component

  return q_res

# Compute the quaternion conjugate of q
# q: 4x1 vector (quaternion)
# ret: 4x1 vector (quaternion)
def qConjugate(q):

  # conj(a + b i + c j + d k) = a - b i - c j - d k
  return np.array([ q[0], -q[1], -q[2], -q[3] ])

# Quaternion Inverse is the conjugate
# q: 4x1 vector (quaternion)
# ret: 4x1 vector (quaternion)
def qInverse(q):

  return qConjugate( q )

# apply a rotation represented by q to a vector v
# v: 3x1 vector
# q: 4x1 vector (quaternion)
def qRotate(v, q):

  pure_v = np.array([ 0.0, v[0], v[1], v[2] ])
  pure_w = qMultiply( qConjugate( q ), qMultiply(pure_v, q))

  return np.array([ pure_w[1], pure_w[2], pure_w[3] ])

# compute the rotation that needs to be applied to go from q1 to q2
# q1: 4x1 vector (quaternion)
# q2: 4x1 vector (quaternion)
# ret: 4x1 vector (quaternion)
def qRotationBetween(q1, q2):

  return qMultiply( q2, qInverse(q1) )
