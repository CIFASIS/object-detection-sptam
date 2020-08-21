#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
Used for comparing a set of runs with different detector/descriptor combinations.
"""

import math
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt 

import latexHelpers as lh
import sptam_parser
import pretty_boxplot

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

IGNORED_SEQUENCES = []

if __name__ == "__main__":

  to_plot = {
    'feature detection' : 'FeatureDetection',
    'descriptor extraction' : 'DescriptorExtraction',
    'extracted points' : 'ExtractedPoints',
    'tracking total' : 'trackingtotal',
  }

  ####################################################################
  # Parse program options
  ####################################################################

  parser = argparse.ArgumentParser()

  parser.add_argument('logfiles', help='list of files containing the logged data', nargs='+')
  args = parser.parse_args()

  ####################################################################
  # Load data
  ####################################################################

  # load pose data. keys are sequence numbers and values are again
  # dictionaries, where keys are extractors and the values are the poses in 3x4 matrix format
  experiments = {}

  for logfile in args.logfiles:

    experiment_config = sptam_parser.loadConfiguration( logfile )

    # get the sequence id from the configuration log
    sequence_id = sptam_parser.loadSequence( experiment_config )

    detector_type = experiment_config[ "detector" ]
    descriptor_type = experiment_config[ "descriptor" ]
    experiment_id = (detector_type, descriptor_type)

    # skip ignored sequences
    if sequence_id in IGNORED_SEQUENCES:
      continue

    if experiment_id not in experiments:
      experiments[ experiment_id ] = {}

    experiments[ experiment_id ][ sequence_id ] = sptam_parser.ExperimentData( logfile, to_plot )

  ####################################################################
  # Plot comparative box plots
  ####################################################################

  #~ for task_label, task_id in to_plot.iteritems():
    #~ data = aggregateOverKey( experiments, lambda det, desc: det+' / '+desc, lambda exp: getattr(exp, task_id)[:,1] )
    #~ pretty_boxplot.boxplot(data.values(), data.keys(), colors, task_label, "required time (s)")

  data = sptam_parser.aggregateOverKey( experiments, lambda (det, desc), seq: det, lambda exp: exp.FeatureDetection[:,1])
  # title should be 'feature detection'
  pretty_boxplot.boxplot(data.values(), data.keys(), colors, None, "required time (s)")
  plt.tight_layout()

  for _, experiment_data in experiments.iteritems():
    for _, sequence_data in experiment_data.iteritems():
      assert( len(sequence_data.DescriptorExtraction[::2,1]+sequence_data.DescriptorExtraction[1::2,1]) == len(sequence_data.ExtractedPoints) )
  normalizeDescriptors = lambda exp: np.divide(exp.DescriptorExtraction[::2,1]+exp.DescriptorExtraction[1::2,1], exp.ExtractedPoints[:,1])
  data = sptam_parser.aggregateOverKey( experiments, lambda (det, desc), seq: desc, normalizeDescriptors )
  # title should be 'normalized descriptor extraction'
  pretty_boxplot.boxplot(data.values(), data.keys(), colors, None, "required time (s)")
  plt.tight_layout()

  data = sptam_parser.aggregateOverKey( experiments, lambda (det, desc), seq: det+' / '+desc, lambda exp: exp.ExtractedPoints[:,1] )
  #~ pretty_boxplot.boxplot(data.values(), data.keys(), colors, 'extracted points', "required time (s)")
  lh.printAvgMaxLatexTable(data, 'extracted points')

  data = sptam_parser.aggregateOverKey( experiments, lambda (det, desc), seq: det+' / '+desc, lambda exp: exp.trackingtotal[:,1] )
  pretty_boxplot.boxplot(data.values(), data.keys(), colors, 'tracking total', "required time (s)")
  lh.printAvgMaxLatexTable(data, 'tracking total')

  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()
