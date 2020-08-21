# -*- coding: utf-8 -*-

import math
import numpy as np

PROB_95=0
PROB_99=1

# for 2D ellipses
chi2_threshold_2D__ = {
  PROB_95: 5.991,
  PROB_99: 9.210
}

# for 3D ellipses
chi2_threshold_3D__ = {
  PROB_95: 7.815,
  PROB_99: 11.345
}

def computeCovarianceRange__(covariance, threshold):

  # A matrix that has real only entries is self adjoint (Hermitian)
  # iif it is symmetric. Since this holds for covariance matrices,
  # we can use a simpler solver.
  w, v = np.linalg.eigh( covariance )

  # sometimes when near the limits floating point precision, an
  # eigenvalue can be negative
  for e in w:
    if ( e < 0 ):
      print("WARNING fixing negative eigenvalue w[0]: ", e)
      e = -e

  # make sure they are in ascending order.
  for e1, e2 in zip(w, w[1:]):
    #~ print(cov, w[0], w[1])
    assert( e1 <= e2 )

  ss = []
  for e in w:
    ss.append( math.sqrt( threshold * e ) )

  return ss, v

def computeCovarianceEllipse(covariance, probability):
  assert(covariance.shape == (2,2))
  return computeCovarianceRange__(covariance, chi2_threshold_2D__[ probability ])

def computeCovarianceEllipsoid(covariance, probability):
  assert(covariance.shape == (3,3))
  return computeCovarianceRange__(covariance, chi2_threshold_3D__[ probability ])
