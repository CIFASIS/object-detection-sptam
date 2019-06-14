#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# import 'print' from python3
from __future__ import print_function

import sys
import os
import argparse
import matplotlib
import parsers.ground_truth_loader as gt_parser

from parsers.sptam import loadPosesWithCovariance

if __name__ == "__main__":

  ##############################################################################
  # parse program options
  ##############################################################################

  parser = argparse.ArgumentParser(description='Plot trajectories computed by S-PTAM.')

  parser.add_argument('logfiles', help='files containing the logged S-PTAM data', nargs='+')

  parser.add_argument('--final', action='store_true', help='Use the final aligned keyframe poses instead of the tracked ones.')
  parser.add_argument('--savetofiles', action='store_true', help='Do not open gui, save plots to files.')

  parser.add_argument('-s', '--start', type=float, help='Filter the data whose elapsed time is greater than or equal to start_offset.')
  parser.add_argument('-e', '--end', type=float, help='Filter the data whose remaining time is greater than end_offset.')

  gt_parser.addProgramOptions( parser, required=True )

  args = parser.parse_args()

  ##############################################################################
  # configure matplotlib
  ##############################################################################

  if (args.savetofiles):
    matplotlib.use('Agg')

  # the rest of graphic libraries need to be imported after the call above
  import matplotlib.pyplot as plt
  import utils.pretty_boxplot as pretty_boxplot
  import utils.plotHelpers as ph
  import utils.mathHelpers as mh
  import utils.error_plotter as error_plotter
  from utils.colors import colors


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
  # load ground truth files
  ##############################################################################

  ground_truth = gt_parser.load( args )

  for _, data in logs.iteritems():
    data.poses = ground_truth.align(data.frame_ids, data.times, data.poses, True)
    # print( len(ground_truth.poses), len(data.poses) )

  ##############################################################################
  # create absolute RMSE errors
  ##############################################################################

  absolute_translation_errors = []
  absolute_rotation_errors = []
  absolute_translation_RMSEs = []
  absolute_rotation_RMSEs = []

  for label, data in logs.iteritems():
    log_name,dummy = os.path.splitext(os.path.basename(label))

    absolute_error_poses = error_plotter.__computeErrorPoses__(ground_truth.poses, data.poses)
    absolute_translation_errors.append( error_plotter.__computeTranslationError__( absolute_error_poses ) )
    absolute_rotation_errors.append( error_plotter.__computeRotationError__( error_plotter.__getRotations__( absolute_error_poses ) ) )

    trans_RMSE = error_plotter.computeRMSE( absolute_translation_errors[-1] )
    rot_RMSE = error_plotter.computeRMSE( absolute_rotation_errors[-1] )

    absolute_translation_RMSEs.append( trans_RMSE )
    absolute_rotation_RMSEs.append( rot_RMSE )

    print(log_name, "\t", "t_abs_rmse:", trans_RMSE, "\t", "rot_abs_rmse:", rot_RMSE)

    #~ ph.plotVsTime1(data.times, absolute_translation_RMSEs[-1])
    #~ ph.plotVsTime1(data.times, absolute_rotation_RMSEs[-1])

    fig1 = pretty_boxplot.boxplot( absolute_translation_errors, labels, colors, "absolute translation", "Euclidean distance (m)" )
    fig2 = pretty_boxplot.boxplot( absolute_rotation_errors, labels, colors, "absolute rotation", "Angular deviation (deg)" )

    if (args.savetofiles):
      fig1.savefig(log_name + '-abs-trans.png')
      fig2.savefig(log_name + '-abs-rot.png')

  ##############################################################################
  # create relative RMSE errors
  ##############################################################################

  relative_translation_errors = []
  relative_rotation_errors = []
  relative_translation_RMSEs = []
  relative_rotation_RMSEs = []

  for label, data in logs.iteritems():
    log_name,dummy = os.path.splitext(os.path.basename(label))

    relative_error_poses = error_plotter.__computeRelativeErrorPoses__(ground_truth.poses, data.poses)
    relative_translation_errors.append( error_plotter.__computeTranslationError__( relative_error_poses ) )
    relative_rotation_errors.append( error_plotter.__computeRotationError__( error_plotter.__getRotations__( relative_error_poses ) ) )

    trans_RMSE = error_plotter.computeRMSE( relative_translation_errors[-1] )
    rot_RMSE = error_plotter.computeRMSE( relative_rotation_errors[-1] )

    relative_translation_RMSEs.append( trans_RMSE  )
    relative_rotation_RMSEs.append( rot_RMSE )

    print(log_name, "\t", "t_rel_rmse:", trans_RMSE, "\t", "rot_rel_rmse:", rot_RMSE)

    fig1 = pretty_boxplot.boxplot( relative_translation_errors, labels, colors, "relative translation", "Euclidean distance (m)" )
    fig2 = pretty_boxplot.boxplot( relative_rotation_errors, labels, colors, "relative rotation", "Angular deviation (deg)" )

    if (args.savetofiles):
      fig1.savefig(log_name + '-rel-trans.png')
      fig2.savefig(log_name + '-rel-rot.png')

  ####################################################################
  # Show all plots
  ####################################################################

  if (not args.savetofiles):
    plt.show()
