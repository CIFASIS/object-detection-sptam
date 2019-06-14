#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import os
import math
import argparse
import collections
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors # colors

import mathHelpers as mh
import sptam_parser
import pretty_boxplot
import comparisonPlotter as comparador
import latexHelpers as lh

colors = [
  matplotlib.colors.colorConverter.to_rgb('forestgreen'),
  matplotlib.colors.colorConverter.to_rgb('red'),
  matplotlib.colors.colorConverter.to_rgb('yellow'),
  matplotlib.colors.colorConverter.to_rgb('blue'),
  matplotlib.colors.colorConverter.to_rgb('magenta'),
  matplotlib.colors.colorConverter.to_rgb('cyan'),
  matplotlib.colors.colorConverter.to_rgb('gray'),
  matplotlib.colors.colorConverter.to_rgb('olive'),
  matplotlib.colors.colorConverter.to_rgb('peru'),
  matplotlib.colors.colorConverter.to_rgb('tomato'),
  matplotlib.colors.colorConverter.to_rgb('chocolate'),
  matplotlib.colors.colorConverter.to_rgb('orange'),
  matplotlib.colors.colorConverter.to_rgb('sienna'),
]

IGNORED_SEQUENCES = ["01"]

#def square( x ):

#  # if it is a numpy array
#  try:
#    ret = x.dot(x)
#  except AttributeError:
#    ret = x*x

#  return ret

#def computeMSE( error_data ):
#  """
#  Compute mean squared error metric
#  """
#  sum_squares = 0.

#  for error_sample in error_data:
#    sum_squares += square(error_sample)

#  return sum_squares / len(error_data)

#def computeRMSE( error_data ):
#  """
#  Compute root-mean-square deviation (RMSD) also called root-mean-square error (RMSE).
#  """
#  return math.sqrt( computeMSE( error_data ) )

def concatOverAllSequences( experiment_data, data_attribute ):

  ret = np.array([])

  for _, pose_data in experiment_data.iteritems():
    data = getattr(pose_data, data_attribute)
    aux = np.hstack((ret, data))
    assert( len(aux) == len(ret) + len(data) )
    ret = aux

  return ret

def computeRMSEOverAllSequences( experiment_data, data_attribute ):

  num_frames = 0
  rmses = 0.

  for sequence_id, pose_data in experiment_data.iteritems():
    data = getattr(pose_data, data_attribute)
    num_frames += len( data )
    rmses += sum([ square(error) for error in data ])
  rmses = math.sqrt( rmses / num_frames )

  return rmses

class readable_dir(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        prospective_dir=values
        if not os.path.isdir( prospective_dir ):
            raise argparse.ArgumentTypeError("readable_dir:{0} is not a valid path".format(prospective_dir))
        if os.access( prospective_dir, os.R_OK ):
            setattr( namespace, self.dest, prospective_dir )
        else:
            raise argparse.ArgumentTypeError("readable_dir:{0} is not a readable dir".format(prospective_dir))

def plotErrorsForEachSequence( experiment_id, sequence_data ):

  # plot the sequences in order
  sorted_data = sorted( sequence_data.items() )

  labels = [ label for label, _ in sorted_data ]

  pretty_boxplot.boxplot( [ pose_data.relative_translation_errors for _, pose_data in sorted_data ], labels, colors, "relative translation errors for "+experiment_id, "Euclidean distance (m)" )
  #~ pretty_boxplot.boxplot( [ pose_data.relative_rotation_errors for _, pose_data in sorted_data ], labels, colors, "relative rotation errors for "+experiment_id, "Angular deviation (deg)" )

def plotErrorsForEachExperiment( experiment_data ):

  relative_translation_errors = {}
  relative_rotation_errors = {}

  for experiment_id, sequence_data in experiment_data.items():

    relative_translation_errors[ experiment_id ] = concatOverAllSequences(sequence_data, "relative_translation_errors")
    relative_rotation_errors[ experiment_id ] = concatOverAllSequences(sequence_data, "relative_rotation_errors")

  pretty_boxplot.boxplot( list( relative_translation_errors.values() ), list( relative_translation_errors.keys() ), colors, None, "Euclidean distance (m)" )
  pretty_boxplot.boxplot( list( relative_rotation_errors.values() ), list( relative_rotation_errors.keys() ), colors, None, "Angular deviation (deg)" )

def extractSequenceLabels( experiment_data ):
  """
  """
  sequence_labels = None

  for experiment_id, sequence_data in experiment_data.items():

    # plot the sequences in order
    sorted_data = sorted( sequence_data.items() )

    if not sequence_labels:
      # take the labels from any experiment, assuming they are the same for all
      sequence_labels = [ sequence_id for sequence_id, _ in sorted_data ]
    else:
      # make sure labels are the same for each experiment
      assert( sequence_labels == [ sequence_id for sequence_id, _ in sorted_data ] )

  return sequence_labels

def plotSequenceDataForEachExperiment( experiment_data, sequence_labels, title ):
  """
  plot a dataset for each experiment, one point per sequence.
  @param sequence_labels list of labels for the sequences (x-axis)
  @param sequence_data dictionary : experiment_id -> sequence_data,
    where sequence data is an array with the same length as sequence_labels
  """
  fig = plt.figure()
  ax = fig.add_subplot(111)

  colors = cm.rainbow(np.linspace(0, 1, len(experiment_data)))

  for (experiment_id, sequence_data), color in zip(experiment_data.items(), colors):
    #~ print sequence_labels, sequence_data
    ax.scatter( sequence_labels, sequence_data, color=color, label=experiment_id )

  recs = []
  for color in colors:
    recs.append(mpatches.Rectangle((0,0),1,1,fc=color))

  #~ handles, labels = ax.get_legend_handles_labels()
  #~ ax.legend(handles, labels, loc='upper left')
  ax.legend(recs, experiment_data.keys())

  ## set labels
  plt.xticks(range(len(sequence_labels)), sequence_labels)

  fig.suptitle( title )

  ax.set_ylabel( "Euclidean distance (m)" )

def plotMaxErrorForEachSequence( experiment_data ):

  sequence_labels = extractSequenceLabels( experiment_data )

  # these dictionaries hold, for each experiment, a list of max error values
  # for each sequence. The dictionary keys are the experiment ids and the error
  # values correspond to the sequences in order, and should have the same length
  # for each.
  absolute_max_translation_errors = {}
  absolute_max_rotation_errors = {}

  for experiment_id, sequence_data in experiment_data.items():

    absolute_max_translation_errors[ experiment_id ] = [ sequence_data[ sequence_id ].absolute_translation_errors.max() for sequence_id in sequence_labels ]
    absolute_max_rotation_errors[ experiment_id ] = [ sequence_data[ sequence_id ].absolute_rotation_errors.max() for sequence_id in sequence_labels ]

  # plot absolute errors
  plotSequenceDataForEachExperiment(absolute_max_translation_errors, sequence_labels, "maximum absolute translation errors")

def plotRelativeRMSEs( experiment_data ):

  relative_translation_errors = sptam_parser.aggregateOverKey( experiment_data, lambda exp_id, seq_id: exp_id, lambda exp: exp.relative_translation_errors )
  relative_rotation_errors = sptam_parser.aggregateOverKey( experiment_data, lambda exp_id, seq_id: exp_id, lambda exp: exp.relative_rotation_errors )

  relative_translation_rmses = sptam_parser.mapDict( relative_translation_errors, comparador.computeRMSE )
  relative_rotation_rmses = sptam_parser.mapDict( relative_rotation_errors, comparador.computeRMSE )

  relative_translation_max = sptam_parser.mapDict( relative_translation_errors, max )
  relative_rotation_max = sptam_parser.mapDict( relative_rotation_errors, max )

  sequence_labels = extractSequenceLabels( experiment_data )

  relative_translation_rmses_by_seq = {}
  for experiment_id, sequence_data in experiment_data.items():
    relative_translation_rmses_by_seq[ experiment_id ] = [ comparador.computeRMSE( sequence_data[ sequence_id ].relative_translation_errors ) for sequence_id in sequence_labels ]

  #~ print "max translation"
#  for experiment_id, value in relative_translation_max.iteritems():
#    print experiment_id, "{:.4f}".format(value)

  #~ print "max rotation"
#  for experiment_id, value in relative_rotation_max.iteritems():
#    print experiment_id, "{:.4f}".format(value)

  for seq_id, rmse in relative_translation_rmses_by_seq.items():
    print( (relative_translation_errors[seq_id])[:5], rmse )

  #~ lh.printMultipleLatexTable([relative_translation_rmses, relative_translation_max], "Relative translation error")
  #~ lh.printMultipleLatexTable([relative_rotation_rmses, relative_rotation_max], "Relative rotation error")
  pretty_boxplot.boxplot(relative_translation_errors.values(), relative_translation_errors.keys(), colors, "Relative translation errors", "Euclidean distance (m)" )
  #~ lh.printLatexTable(relative_translation_rmses_by_seq, sequence_labels, "Relative translation error")
  plotSequenceDataForEachExperiment(relative_translation_rmses_by_seq, sequence_labels, "Relative translation RMSE")

def plotAbsoluteRMSEs( experiment_data ):

  sequence_labels = extractSequenceLabels( experiment_data )

  absolute_translation_rmses = {}
  absolute_rotation_rmses = {}
  absolute_translation_rmses_by_seq = {}

  for experiment_id, sequence_data in experiment_data.items():

    absolute_translation_rmses_by_seq[ experiment_id ] = [ comparador.computeRMSE( sequence_data[ sequence_id ].absolute_translation_errors ) for sequence_id in sequence_labels ]

    absolute_translation_rmses[ experiment_id ] = comparador.computeRMSE(concatOverAllSequences( sequence_data, "absolute_translation_errors") )
    absolute_rotation_rmses[ experiment_id ] = comparador.computeRMSE(concatOverAllSequences( sequence_data, "absolute_rotation_errors") )

  lh.print2DLatexTable(absolute_translation_rmses, "Absolute translation error")
  lh.print2DLatexTable(absolute_rotation_rmses, "Absolute rotation error")
  #~ lh.printLatexTable(absolute_translation_rmses_by_seq, sequence_labels, "Absolute translation error")
  plotSequenceDataForEachExperiment(absolute_translation_rmses_by_seq, sequence_labels, "Absolute root mean squared error")

if __name__ == "__main__":

  ##############################################################################
  # parse program options
  ##############################################################################

  parser = argparse.ArgumentParser()

  parser.add_argument('sptam', action=readable_dir, help='folder containing a folder for each sequence, each one with a file for each experiment.')
  parser.add_argument('grnd', action=readable_dir, help='folder containing the KITTI ground truth poses for each sequence on a separate file.')
  parser.add_argument('--dataset', default=sptam_parser.implemented_datasets[0], choices=sptam_parser.implemented_datasets, help='dataset to be parsed.')

  args = parser.parse_args()

  ##############################################################################
  # load ground truth data
  ##############################################################################

  # create dictionary for ground truth poses. The keys
  # are the sequence id's, the values are the poses in 3x4 matrix format
  ground_truth_poses = {}
  ground_truth_timestamps = {}

  for filename in os.listdir( args.grnd ):
    if filename.endswith( ".txt" ):

      sequence_id = os.path.splitext( filename )[0]

      # skip ignored sequences
      if sequence_id in IGNORED_SEQUENCES:
        continue

      file_path = os.path.join(args.grnd, filename)

      ground_truth_timestamps[ sequence_id ], ground_truth_poses[ sequence_id ] = sptam_parser.loadGroundTruth(args.dataset, file_path)

      print "loaded ground truth file for sequence", sequence_id

  ##############################################################################
  # load experiment data
  ##############################################################################

  # load pose data. keys are sequence numbers and values are again
  # dictionaries, where keys are extractors and the values are the poses in 3x4 matrix format
  experiment_poses = {}

  for filename in os.listdir( args.sptam ):
    if filename.endswith( ".log" ):

      file_path = os.path.join(args.sptam, filename)

      experiment_config = sptam_parser.loadConfiguration( file_path )

      # get the sequence id from the configuration log
      sequence_id = sptam_parser.loadSequence(experiment_config)

      # skip ignored sequences
      if sequence_id in IGNORED_SEQUENCES:
        continue

      detector_type = experiment_config[ "detector" ]
      descriptor_type = experiment_config[ "descriptor" ]

      ######################################################################
      # For feature extractor comparison experiments
      experiment_id = detector_type + ' / ' + descriptor_type
      ######################################################################

      #~ ######################################################################
      #~ # For window size comparison experiments
#      experiment_id = experiment_config[ "BundleAdjustmentActiveKeyframes" ]
#      experiment_id = int(experiment_id)
      #~ ######################################################################

      print "loading experiment for sequence", sequence_id, "with label:", "'"+str(experiment_id)+"'"

      # ignore the firs column (timestamp) for the sptam poses
      sptam_pose_data = sptam_parser.loadPoses( file_path )
      sptam_timestamps = sptam_pose_data[:,0]
      sptam_poses = sptam_pose_data[:,1:13]
      sptam_poses = sptam_poses.reshape(len(sptam_poses),3,4)

      if experiment_id not in experiment_poses:
        experiment_poses[ experiment_id ] = {}

      assert( sequence_id not in experiment_poses[ experiment_id ] )

      experiment_poses[ experiment_id ][ sequence_id ] = sptam_parser.loadPoseData(args.dataset, sptam_timestamps, sptam_poses, ground_truth_timestamps[ sequence_id ], ground_truth_poses[ sequence_id ], experiment_id)

  ##############################################################################
  # Plot data
  ##############################################################################

  plotRelativeRMSEs( experiment_poses )
  #~ plotAbsoluteRMSEs( experiment_poses )

  # load relative errors for each sequence
  #~ plotErrorsForEachExperiment( experiment_poses )

  # load absolute errors for each descriptor / sequence
  #~ plotMaxErrorForEachSequence( experiment_poses )

  #~ for experiment_id, sequence_data in experiment_poses.items():
    #~ plotErrorsForEachSequence( experiment_id, sequence_data )

  plt.show()
