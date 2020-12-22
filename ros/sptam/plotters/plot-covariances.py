#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import parsers.sptam as data_parser
import mathHelpers as mh
import covariance_ellipsoid

# for 2D ellipses
CHI2_S_95 = 5.991
CHI2_S_99 = 9.210

# for 3D ellipses
CHI2_S_3_95 = 7.815
CHI2_S_3_99 = 11.345

# src: http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

"""
  fix data in the camera convention frame, where z points forward
  and y points down, so that x points forward and z up.
"""
frame_fix = np.array([
  [ 0,  0,  1],
  [-1,  0,  0],
  [ 0, -1,  0],
])

def getCovarianceVolume( cov, scale ):

  # ellipsoid volume = 4/3 π abc
  # where a, b and c are the half-lengths of the principal axes.

  # The principal axe sizes for a covariance matrix are sqrt( w_i * s )
  # where w_i is the i-th eigenvalue and s the ellipse scale.
  # Therefore the volume becomes 4/3 π sqrt(w0*s) sqrt(w1*s) sqrt(w2*s)
  # which equals 4/3 π sqrt(s*s*s) sqrt(w0*w1*w2)

  # The product of all the eigenvalues of A = det(A)
  # src: http://www.real-statistics.com/linear-algebra-matrix-topics/eigenvalues-eigenvectors/

  #This finally gives us a volume = 4/3 π sqrt(s*s*s) sqrt( det(cov) )

  return (4./3.) * math.pi * math.sqrt( scale*scale*scale ) * math.sqrt( np.linalg.det( cov ) )

def getPrincipalAxes( cov, scale ):
  """
  @brief get the principal axes of the 2D p-ellipsoid of a variance-covariance matrix.
  """

  ws, vs = np.linalg.eigh( cov )

  # sometimes when near the limits  floating point precision, an
  # eigenvalue can be negative

  for w in ws:
    if ( w < 0 ):
      print("WARNING fixing negative eigenvalue w[0]: ", w)
      w = -w

  # make sure they are in ascending order.
  for w1, w2 in zip(ws, ws[1:]):
    #~ print(cov, w[0], w[1])
    assert( w1 <= w2 )

  # scale the axes to represent n-percent probability
  ss = []
  for w in ws:
    ss.append( 2 * math.sqrt( scale * w ) )

  return vs, ss

# each sample in data should be an array of 6 elemnts
# [x, y, s00, s01, s10, s11] where S = [[s00, s01], [s10, s11]] is the
# covariance matrix for x and y in row major order.
def plotEllipses2D( positions, covariances ):

  fig = plt.figure()
  ax = fig.add_subplot(111, aspect='equal')

  for pos, cov in zip(positions, covariances):

    ss = cov[:2, :2]

    s, v = covariance_ellipsoid.computeCovarianceEllipse(ss, covariance_ellipsoid.PROB_99)

    # compute angle of the "largest" eigenvector.
    angle = math.atan2( v[0][1], v[0][0] )

    e = Ellipse(xy=(pos[0], pos[1]), width=s[0], height=s[1], angle=math.degrees( angle ), alpha=.75)
    ax.add_artist( e )

  xs = positions[:,0]
  ys = positions[:,1]
  ax.scatter(xs, ys, alpha=.1, color='r')

  ax.set_ylabel("z (m)")
  ax.set_xlabel("x (s)")

  ax.grid( True )

  # KITTI 00
  #~ ax.set_xlim(-50, 500)
  #~ ax.set_ylim(-300, 300)

  ax.set_xlim(-500, 500)
  ax.set_ylim(-500, 500)

  next

def plotCovByAxis( data ):

  ts = range(len(data))

  xs = data[:,0]
  ys = data[:,1]
  sxs = np.sqrt( data[:,2] * CHI2_S_95 )
  sys = np.sqrt( data[:,5] * CHI2_S_95 )

  fig, (ax_x, ax_y) = plt.subplots(2, sharex=True, sharey=True)

  ax_x.fill_between(ts, xs+sxs, xs-sxs, color='gray', alpha=0.5)
  ax_x.plot( ts, xs, 'r' )

  ax_y.fill_between(ts, ys+sys, ys-sys, color='gray', alpha=0.5)
  ax_y.plot( ts, ys, 'r' )

def plotValueAndCov( xs, ss ):

  ts = range(len(data))

  ss = np.sqrt( ss * CHI2_S_95 )

  fig, (ax_x, ax_s) = plt.subplots(2, sharex=True, sharey=False)

  ax_x.plot( ts, xs, 'r' )
  ax_s.plot( ts, ss, 'b--' )

def loadData( filename ):

  f = open(args.filename, 'r')
  lines = f.readlines()

  # 19 lines make up a message, we are interested in some of them
  assert( len(lines) % 19 == 0 )

  # compute number of recorded messages
  n_msgs = len(lines) / 19

  ret = []

  for i in range( n_msgs ):

    x_str = lines[ i*19 + 9 ].split()
    assert( len(x_str) == 2 )
    assert( x_str[0] == 'x:' )

    y_str = lines[ i*19 + 10 ].split()
    assert( len(y_str) == 2 )
    assert( y_str[0] == 'y:' )

    covs = map(float, lines[ i*19 + 17 ][15:-2].split(', '))
    assert( len(covs) == 36 )
    cov_matrix = np.array(covs).reshape((6,6))

    data = [ x_str[-1], y_str[-1], cov_matrix[0][0], cov_matrix[0][1], cov_matrix[1][0], cov_matrix[1][1] ]

    ret.append( data )

  return np.array( ret ).astype(np.float)

def transformToCameraFrame(Rs, covs):
  """
  @brief transform a set of covariances into the camera frame.
  """

  assert( len(Rs) == len(covs) )

  return np.array([ R.dot( cov ).dot( R.transpose() ) for cov, R in zip(covs, Rs) ])

def plotCovarianceByAxis3D(idxs, covs, scale, title):
  """
  @brief plot the individual axis variances, in the camera refernce frame.
  """

  assert( len(idxs) == len(covs) )

  sxs = np.sqrt( covs[:,0,0] * scale )
  sys = np.sqrt( covs[:,1,1] * scale )
  szs = np.sqrt( covs[:,2,2] * scale )

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  ax_x.scatter( idxs, sxs )
  ax_y.scatter( idxs, sys )
  ax_z.scatter( idxs, szs )

  ax_x.grid(True)
  ax_y.grid(True)
  ax_z.grid(True)

  fig.suptitle( title )

def plotCovarianceRatioByAxis3D(idxs, covs, scale, title):
  """
  @brief plot the individual axis variances, in the camera refernce frame.
  """

  assert( len(idxs) == len(covs) )

  sxs = np.sqrt( covs[:,0,0] * scale )
  sys = np.sqrt( covs[:,1,1] * scale )
  szs = np.sqrt( covs[:,2,2] * scale )

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  ax_x.scatter( idxs, np.divide(sxs, sys) )
  ax_y.scatter( idxs, np.divide(sxs, szs) )
  ax_z.scatter( idxs, np.divide(sys, szs) )

  ax_x.grid(True)
  ax_y.grid(True)
  ax_z.grid(True)

  fig.suptitle( title )

def plotCovarianceHistogramByAxis3D(idxs, covs, scale, title):

  assert( len(idxs) == len(covs) )

  sxs = np.sqrt( covs[:,0,0] * scale )
  sys = np.sqrt( covs[:,1,1] * scale )
  szs = np.sqrt( covs[:,2,2] * scale )

  #~ fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  #~ ax_x.hist(sxs, 300)
  #~ ax_y.hist(sys, 300)
  #~ ax_z.hist(szs, 300)

  #~ ax_x.grid(True)
  #~ ax_y.grid(True)
  #~ ax_z.grid(True)

  #~ fig.suptitle( title )

  # all in one

  fig = plt.figure()
  ax = fig.add_subplot(111)

  data = np.vstack([sxs, sys, szs]).T
  ax.hist(data, 1000, label=['x', 'y', 'z'])

  plt.legend(loc='upper right')
  fig.suptitle( title )

def plotCovarianceByPrincipalAxis3D(idxs, covs, scale, title):
  """
  @brief plot the individual axis variances, in the camera refernce frame.
  """
  assert( len(idxs) == len(covs) )

  lengths = np.array([ getPrincipalAxes( cov, scale )[1] for cov in covs ])

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  ax_x.plot( idxs, lengths[:,0] )
  ax_y.plot( idxs, lengths[:,1] )
  ax_z.plot( idxs, lengths[:,2] )

  fig.suptitle( title )

def plotCovarianceVolume3D(idxs, covs, scale, title):
  """
  @brief plot the volume of each covariance ellipse, it should be a good measure of precision.
  """

  assert( len(idxs) == len(covs) )

  volumes = [ getCovarianceVolume( cov, scale ) for cov in covs ]

  fig = plt.figure()
  ax = fig.add_subplot(111)

  ax.plot( idxs, volumes )

  fig.suptitle( title )

def applyTransform(transform, ts, Rs, pos_cov, ori_cov):
  new_ts = np.array([ transform.dot( elem ) for elem in ts ])
  new_Rs = np.array([ transform.dot( elem ) for elem in Rs ])
  new_pos_cov = np.array([ transform.dot( elem ).dot( transform.transpose() ) for elem in pos_cov ])
  new_ori_cov = np.array([ transform.dot( elem ).dot( transform.transpose() ) for elem in ori_cov ])
  return new_ts, new_Rs, new_pos_cov, new_ori_cov

if __name__ == "__main__":

  # parse program options

  parser = argparse.ArgumentParser()

  parser.add_argument('filename', help='file containing the logged data')
  parser.add_argument('--align', dest='align', action='store_true', help='fix data in the camera convention frame, where z points forward and y points down, so that x points forward and z up.')

  args = parser.parse_args()

  ####################################################################
  # Load data
  ####################################################################

  frames, poses, covariances = data_parser.loadPosesWithCovariance( args.filename )
  frames = frames.astype(int)

  Rs, ts = mh.decomposeTransformations( poses )

  pos_cov = covariances[:,:3,:3]
  ori_cov = covariances[:,3:,3:]

  ####################################################################
  # Align data
  ####################################################################

  if args.align:
    ts, Rs, pos_cov, ori_cov = applyTransform(frame_fix, ts, Rs, pos_cov, ori_cov)

  ####################################################################
  # plot covariance ellipses
  ####################################################################

  plotEllipses2D( ts, pos_cov )

  pos_cov_cam = transformToCameraFrame(Rs, pos_cov)
  ori_cov_cam = transformToCameraFrame(Rs, ori_cov)

  # position covariance plots
  plotCovarianceByAxis3D(frames, pos_cov_cam, CHI2_S_3_95, "position 95-p range by camera axis")
  plotCovarianceRatioByAxis3D(frames, pos_cov_cam, CHI2_S_3_95, "position covariance ratio by camera axis")
  plotCovarianceHistogramByAxis3D(frames, pos_cov_cam, CHI2_S_3_95, "position 95-p range histogram by camera axis")
  #~ plotCovarianceByPrincipalAxis3D(frames, pos_cov_cam, CHI2_S_3_95, "position 95-p range by principal axis")
  #~ plotCovarianceVolume3D(frames, pos_cov_cam, CHI2_S_3_95, "position covariance volume")

  # orientation covariance plots
  #~ plotCovarianceByAxis3D(frames, ori_cov_cam, CHI2_S_3_95, "orientation 95-p range by camera axis")
  #~ plotCovarianceByPrincipalAxis3D(frames, ori_cov_cam, CHI2_S_3_95, "orientation 95-p range by principal axis")
  #~ plotCovarianceVolume3D(frames, ori_cov_cam, CHI2_S_3_95, "orientation covariance volume")

  #~ plotCovByAxis( data )
  #~ plotValueAndCov( data[:,0], data[:,2] )
  #~ plotValueAndCov( data[:,1], data[:,5] )

  plt.show()
