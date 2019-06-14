#!/usr/bin/env python2
# -*- coding: utf-8 -*- 

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

# add utils to the python library path.
# This way submodules in other folders can acces it.
sys.path.insert(1, 'utils')

import mathHelpers as mh
import plotHelpers as ph
import utils.error_plotter as error_plotter
from colors import colors, ground_truth_color

from parsers.sptam import loadPosesWithCovariance

def plotLinesByAxis(data, colors, title=""):

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  for (ts, xs, label), color in zip(data, colors):
    ax_x.scatter(ts, xs[:,0], c=color, label=label)
    ax_y.scatter(ts, xs[:,1], c=color, label=label)
    ax_z.scatter(ts, xs[:,2], c=color, label=label)

  # Fine-tune figure; make subplots close to each other and hide x ticks for
  # all but bottom plot.
  #fig.subplots_adjust(hspace=0)
  plt.setp([a.get_xticklabels() for a in fig.axes[:-1]], visible=False)

  ax_z.set_xlabel("frame number")

  ax_x.grid( True )
  ax_y.grid( True )
  ax_z.grid( True )

  handles, labels = ax_x.get_legend_handles_labels()
  ax_x.legend(handles, labels, loc='upper left')

  #~ fig.tight_layout()

  fig.suptitle( title )

if __name__ == "__main__":

  ##############################################################################
  # parse program options
  ##############################################################################

  parser = argparse.ArgumentParser()

  parser.add_argument('logfile', help='file containing the logged data')

  args = parser.parse_args()

  ##############################################################################
  # prepare some variables
  ##############################################################################

  ##############################################################################
  # Load Pose data
  ##############################################################################

  # Each poses object will have the following attributes: frame_ids, times, poses, covariances

  estimations = loadPosesWithCovariance(args.logfile, "ESTIMATED_CAMERA_POSE")
  refinations = loadPosesWithCovariance(args.logfile, "REFINED_CAMERA_POSE")

  assert( len(estimations.frame_ids) == len(refinations.frame_ids) )

  estimation_orientation_matrices, estimation_positions = mh.decomposeTransformations( estimations.poses )
  refination_orientation_matrices, refination_positions = mh.decomposeTransformations( refinations.poses )

  estimation_orientations_euler = np.array([ mh.eulerAnglesfromRotationMatrix( R ) for R in estimation_orientation_matrices ])
  refination_orientations_euler = np.array([ mh.eulerAnglesfromRotationMatrix( R ) for R in refination_orientation_matrices ])

  ####################################################################
  # Plot Prediction VS Updates
  ####################################################################

  plotLinesByAxis( [
    (estimations.frame_ids, estimation_positions, "estimated"),
    (refinations.frame_ids, refination_positions, "refined")
  ], colors, "position predictions vs. updates" )

  #~ plt.savefig('plot3.png')

  plotLinesByAxis( [
    (estimations.frame_ids, estimation_orientations_euler, "estimated"),
    (refinations.frame_ids, refination_orientations_euler, "refined")
  ], colors, "orientation predictions vs. updates" )

  #~ plt.savefig('plot3.png')

  ####################################################################
  # Plot position errors by axis and absolute
  ####################################################################

  position_errors_by_axis = refination_positions - estimation_positions
  position_errors = error_plotter.__computeAbsoluteTranslationErrors__(estimation_positions, refination_positions)

  error_plotter.plotErrorsByAxis(estimations.frame_ids, position_errors_by_axis, "Position prediction error", None, ("x (m)", "y (m)", "z (m)"))
  ph.plotVsTime1(estimations.frame_ids, position_errors, "Position prediction error", None, "(m)")

  ####################################################################
  # Plot orientation errors by axis and absolute
  ####################################################################

  orientation_error_matrices = error_plotter.__computeRotationErrorMatrices__(estimation_orientation_matrices, refination_orientation_matrices)
  orientation_errors_by_axis = np.array([ mh.eulerAnglesfromRotationMatrix( R ) for R in orientation_error_matrices ])
  orientation_errors = error_plotter.__computeRotationError__( orientation_error_matrices )

  error_plotter.plotErrorsByAxis(estimations.frame_ids, orientation_errors_by_axis, "Orientation prediction error", None, ("x (deg)", "y (deg)", "z (deg)"))
  ph.plotVsTime1(estimations.frame_ids, orientation_errors, "Orientation prediction error", None, "(deg)")

  ####################################################################
  # Show all
  ####################################################################

  plt.show()
