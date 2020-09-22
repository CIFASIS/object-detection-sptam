#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
import math
import bisect
import argparse
import transformations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#~ colors = [
  #~ (0, 0.4470, 0.7410),
  #~ (0.8500, 0.3250, 0.0980),
  #~ (0.9290, 0.6940, 0.1250),
  #~ (0.4940, 0.1840, 0.5560),
  #~ (0.4660, 0.6740, 0.1880),
  #~ (0.3010, 0.7450, 0.9330),
  #~ (0.6350, 0.0780, 0.1840),
  #~ (0.6290, 0.4940, 0.1250),
  #~ (0.4660, 0.6740, 0.5880)
#~ ]

colors = [
  (1, 0, 0),
  (0, 1, 0),
  (0, 0, 1),
  (1, 1, 0),
  (0, 1, 1),
  (1, 0, 1),
]

def composeTransformation( R, t ):

  T = np.eye(4, 4)

  T[0:3,0:3] = R
  T[:3,3] = t

  return T

def composeTransformationOP( O, p ):
  
  return composeTransformation( O.transpose(), -O.transpose().dot( p ) )

def decomposeTransformation( T ):

  #~ print(len(transform))
  P = np.reshape( T, (3, 4) )

  R = P[0:3,0:3]
  t = P[:,3]

  return R, t

def decomposeTransformations( transformations ):

  Rs = []
  ts = []

  for transform in transformations:

    R, t = decomposeTransformation( transform )

    Rs.append( R )
    ts.append( t )

  return np.array( Rs ), np.array( ts )

def alignInitialFrame( poss, oris, pos_to, ori_to ):

  src_inv = composeTransformationOP( oris[0], poss[0] )
  dst = composeTransformation( ori_to, pos_to )

  #~ src_inv = composeTransformation( oris[0], poss[0] )
  #~ dst = composeTransformation( ori_to.transpose(), -ori_to.transpose().dot( pos_to ) )

  R, t = decomposeTransformation( ( dst.dot( src_inv ) )[:3,:] )

  poss_new = []
  for pos in poss:
    pos_new = R.dot( pos ) + t
    poss_new.append( pos_new )

  oris_new = []
  for ori in oris:
    ori_new = R.dot( ori )
    oris_new.append( ori_new )

  # convert data to numpy arrays
  return np.array( poss_new ), np.array( oris_new )

def plot3D( datas, title ):

  min_val = 0
  max_val = 0

  # simulate equal aspect ratio
  for data in datas:
    min_val = min(min_val, data[:,0].min(), data[:,1].min(), data[:,2].min())
    max_val = max(max_val, data[:,0].max(), data[:,1].max(), data[:,2].max() )
  
  fig = plt.figure()

  ax = fig.add_subplot(111, projection='3d')

  for data, color in zip(datas, colors):
    ax.plot(data[:,0], data[:,1], data[:,2], c=color)

  # simulate equal aspect ratio
  #~ """
  #~ max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
  #~ mean_x = X.mean()
  #~ mean_y = Y.mean()
  #~ mean_z = Z.mean()
  ax.set_xlim(min_val, max_val)
  ax.set_ylim(min_val, max_val)
  ax.set_zlim(min_val, max_val)
  #~ """
  # Set axis labels

  xLabel = ax.set_xlabel('x')
  yLabel = ax.set_ylabel('y')
  zLabel = ax.set_zlabel('z')

  ax.set_title( title )

################################################################################
# MAIN
################################################################################

# grep ^BASE_LINK_POSE sarasa.log | awk '{print $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12, $13, $14}' > test_full.log

if __name__ == "__main__":

  ####################################################################
  # parse program options
  ####################################################################

  parser = argparse.ArgumentParser()
  parser.add_argument('logfiles', help='list of log files with format [t x y z R00 R01 .. R22] : Nx13', nargs='+')
  args = parser.parse_args()

  ####################################################################
  # Load data
  ####################################################################

  sequences = []
  timestamps = []

  for logfile in args.logfiles:

    print(logfile)
    data = np.loadtxt( logfile )
    # make sure data has shape N x 13
    assert( data.shape[1] == 1+3+9+1 )
    ori, pos = decomposeTransformations( data[:,1:(1+3+9)] )
    sequences.append( (pos, ori) )
    timestamps.append( data[:,0] )

  unalign = False
  if unalign:

    ####################################################################
    # plot original sequences
    ####################################################################

    tracks = []
    for seq in sequences:
      tracks.append( seq[0] )

    plot3D( tracks, "original tracks" )

    ####################################################################
    # disalign sequences
    ####################################################################

    #~ O_ini = np.eye(3,3)
    #~ p_ini = np.zeros(3)

    O_ini = sequences[0][1][0]
    p_ini = sequences[0][0][0]

    aux = []
    for poss, oris in sequences:
      poss_new, oris_new = alignInitialFrame( poss, oris, p_ini, O_ini )
      aux.append( (poss_new, oris_new) )

    sequences = aux

  ####################################################################
  # plot unaligned sequences
  ####################################################################

  #~ print("unaligned sequences:", len(sequences))
  tracks = []
  for poss, oris in sequences:
    tracks.append( poss )

  plot3D( tracks, "unaligned tracks" )

  #~ plt.show()
  #~ sys.exit(1)

  ####################################################################
  # Align sequences
  ####################################################################

  sequences_aligned = []
  sequences_aligned.append( sequences[0] )

  O_last = sequences[0][1][-1]
  p_last = sequences[0][0][-1]

  for poss, oris in sequences[1:]:
    poss_new, oris_new = alignInitialFrame( poss, oris, p_last, O_last )
    sequences_aligned.append( (poss_new, oris_new) )
    O_last = oris_new[-1]
    p_last = poss_new[-1]

  ####################################################################
  # plot aligned sequences
  ####################################################################

  tracks = []
  for poss, oris in sequences_aligned:
    tracks.append( poss )

  plot3D( tracks, "aligned tracks" )

  ####################################################################
  # make sure ...
  ####################################################################

  for ts1, ts2 in zip(timestamps[:-1], timestamps[1:]):
    print(ts1[-1], ts2[0])

  ####################################################################
  # Save one concatenated file
  ####################################################################

  sequences_concatenated = []

  poss_first, oris_first = sequences_aligned[0]
  for pos, ori, time in zip(poss_first, oris_first, timestamps[0]):
    #~ print( pos[0], pos[1], pos[2], ori[0][0], ori[0][1], ori[0][2], ori[1][0], ori[1][1], ori[1][2], ori[2][0], ori[2][1], ori[2][2] )
    sequences_concatenated.append( np.array([ time, ori[0][0], ori[0][1], ori[0][2], pos[0], ori[1][0], ori[1][1], ori[1][2], pos[1], ori[2][0], ori[2][1], ori[2][2], pos[2] ]) )

  for (poss, oris), times in zip(sequences_aligned[1:], timestamps[1:]):
    for pos, ori, time in zip(poss, oris, times)[1:]:
      #~ print( pos[0], pos[1], pos[2], ori[0][0], ori[0][1], ori[0][2], ori[1][0], ori[1][1], ori[1][2], ori[2][0], ori[2][1], ori[2][2] )
      sequences_concatenated.append( np.array([ time, ori[0][0], ori[0][1], ori[0][2], pos[0], ori[1][0], ori[1][1], ori[1][2], pos[1], ori[2][0], ori[2][1], ori[2][2], pos[2] ]) )

  np.savetxt('full_sequence.txt', sequences_concatenated, delimiter=" ")

  ####################################################################
  # show all plots
  ####################################################################

  plt.show()
