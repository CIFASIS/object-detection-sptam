#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

# add utils to the python library path.
# This way submodules in other folders can acces it.
sys.path.insert(1, 'utils')

import plotHelpers as ph
import mathHelpers as mh
from colors import colors, ground_truth_color
import parsers.ground_truth_loader as gt_parser

from parsers.sptam import loadPosesWithCovariance

if __name__ == "__main__":

  ##############################################################################
  # parse program options
  ##############################################################################

  parser = argparse.ArgumentParser(description='Plot trajectories computed by S-PTAM.')

  parser.add_argument('logfiles', help='files containing the logged S-PTAM data', nargs='+')

  parser.add_argument('--final', action='store_true', help='Plot the final aligned keyframe poses instead of the tracked ones.')

  parser.add_argument('-s', '--start', type=float, help='Filter the data whose elapsed time is greater than or equal to start_offset.')
  parser.add_argument('-e', '--end', type=float, help='Filter the data whose remaining time is greater than end_offset.')

  gt_parser.addProgramOptions( parser )

  args = parser.parse_args()

  ##############################################################################
  # prepare some variables
  ##############################################################################

  pose_tag = "TRACKED_FRAME_POSE" if not args.final else "FINAL_FRAME_POSE"

  labels = args.logfiles

  ##############################################################################
  # load pose logs
  ##############################################################################

  # Each poses object will have the following attributes: frame_ids, times, poses, covariances

  logs = {}
  for label, logfile in zip(labels, args.logfiles):
    logs[ label ] = loadPosesWithCovariance(logfile, pose_tag, args.start, args.end)

  ##############################################################################
  # load ground truth files and align data to it
  ##############################################################################

  ground_truth = gt_parser.load( args )

  if ground_truth:

    for _, data in logs.iteritems():
      data.poses = ground_truth.align(data.frame_ids, data.times, data.poses)

      labels = np.append("ground truth", labels)
      colors = np.vstack((ground_truth_color, colors))

  ##############################################################################
  # sepparate position and orientation
  ##############################################################################

  for label, data in logs.iteritems():
    ori, pos = mh.decomposeTransformations( data.poses )
    data.positions = pos
    data.orientations = ori

  ##############################################################################
  # create path plots
  ##############################################################################

  path_xy = []
  path_xz = []
  path_yz = []
  path_3d = []

  if ground_truth:
    _, gt_positions = mh.decomposeTransformations( ground_truth.poses )
    path_xy.append( ( gt_positions[:,0], gt_positions[:,1] ) )
    path_xz.append( ( gt_positions[:,0], gt_positions[:,2] ) )
    path_yz.append( ( gt_positions[:,1], gt_positions[:,2] ) )
    path_3d.append( ( gt_positions[:,0], gt_positions[:,1], gt_positions[:,2] ) )

  for label, data in logs.iteritems():
    path_xy.append( ( data.positions[:,0], data.positions[:,1] ) )
    path_xz.append( ( data.positions[:,0], data.positions[:,2] ) )
    path_yz.append( ( data.positions[:,1], data.positions[:,2] ) )
    path_3d.append( ( data.positions[:,0], data.positions[:,1], data.positions[:,2] ) )

  ph.plotPaths2D( path_xy, labels, colors, xlabel="x (m)", ylabel="y (m)" )
  ph.plotPaths2D( path_xz, labels, colors, xlabel="x (m)", ylabel="z (m)" )
  ph.plotPaths2D( path_yz, labels, colors, xlabel="y (m)", ylabel="z (m)" )
  ph.plotPaths3D( path_3d, labels, colors )

  #~ plt.savefig("fig.png", bbox_inches='tight', transparent=True, dpi=200)

  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()
