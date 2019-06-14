#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np

# Functions to print latex tables

def printLatexTable(experiment_data, sequence_labels, title):

  # make data matrix and labels

  data_matrix = []
  for experiment_id, sequence_data in experiment_data.items():
    data_matrix.append( np.array(sequence_data) )
  data_matrix = np.array( data_matrix ).transpose()

  sequence_labels = [ sequence_id for sequence_id in sequence_labels ]
  experiment_labels = [ "win "+str(experiment_id) for experiment_id in experiment_data ]

  ##############################################################################

  assert( data_matrix.shape[0] == len(sequence_labels) )
  assert( data_matrix.shape[1] == len(experiment_labels) )

  # number of data columns
  cols = data_matrix.shape[1]

  print "\\begin{tabular}{|c|*{"+str(cols)+"}{c}|}"
  print "  \hline"
  print "  \multicolumn{"+str(cols+1)+"}{|c|}{"+title+"} \\\\"
  print "  \hline"

  header = "  " + "seq"
  for experiment_id in experiment_labels:
    header += " & " + experiment_id
  header += " \\\\"
  print header

  print "  \hline \hline"

  for sequence_id, sequence_data in zip(sequence_labels, data_matrix):
    line = "  " + sequence_id
    min_elem = min(sequence_data)
    for sample in sequence_data:
      line += " & "
      if sample == min_elem:
        line += "\\textbf{"
      line += "{:.2f}".format(sample)
      if sample == min_elem:
        line += "}"
    line += " \\\\"
    print line

  print "  \hline"
  print "\end{tabular}"

def print2DLatexTable(experiment_data, title):

  sorted_data = sorted(experiment_data.iteritems(), key=lambda (k,v): (v,k))

  ##############################################################################

  print "\\begin{tabular}{|c|c|}"
  print "  \hline"
  print "  \multicolumn{2}{|c|}{"+title+"} \\\\"
  print "  \hline"

  # header
  print "  exp & value \\\\"
  print "  \hline \hline"

  for key, value in sorted_data:
    #~ print key, value
    print "  " + key + " & " + "{:.4f}".format(value) + " \\\\"

  print "  \hline"
  print "\end{tabular}"


def printMultipleLatexTable(experiment_data_list, title):

  # number of experiments to process
  experiments_number = len(experiment_data_list)

  # sort the data following the first array
  experiment_data1 = experiment_data_list[0]
  sorted_data1 = sorted(experiment_data1.iteritems(), key=lambda (k,v): (v,k))

  ##############################################################################

  # columns are all the experiments plus the column of experiments' labels
  columns_number = experiments_number + 1

  # header of the Latex Table
  print "\\begin{tabular}{|" + 'c|' * columns_number + "}" # create columns
  print "  \hline"
  print "  \multicolumn{"+ str(columns_number) + "}{|c|}{"+title+"} \\\\"
  print "  \hline"
  print "  exp " + '& column ' * columns_number + "\\\\"
  print "  \hline \hline"


  # construct the table iterating over the first experiment
  for key, value in sorted_data1:

    row = key # the first element of each row is the label of the experiment
    for experiment_data in experiment_data_list:
      row = row + " & " + "{:.4f}".format(experiment_data[key])

    print "  " + row + " \\\\"

  print "  \hline"
  print "\end{tabular}"


def printDictAsLatexTable(dictionary, title, float_format="{:.4f}"):

  data = sorted(dictionary.iteritems(), key=lambda (k,v): (k,v))

  print "\\begin{tabular}{|c|c|}"
  print "  \hline"
  print "  \multicolumn{2}{|c|}{"+title+"} \\\\"
  print "  \hline"

  # header
  print "  key & value \\\\"
  print "  \hline \hline"

  for key, value in data:
    print "  " + str(key) + " & " + float_format.format(value) + " \\\\"

  print "  \hline"
  print "\end{tabular}"

def printAvgMaxLatexTable(experiment_data, title, sort=True, float_format="{:.4f}"):

  avg_max_values = {}

  for key, values in experiment_data.iteritems():

    avg_value = np.mean( values )
    max_value = max( values )

    avg_max_values[ key ] = (avg_value, max_value)

  if sort:
    avg_max_values = sorted(avg_max_values.iteritems(), key=lambda (k,(avg_value, max_value)): avg_value)

  print "\\begin{tabular}{|c|c|c|}"
  print "  \hline"
  print "  \multicolumn{3}{|c|}{"+title+"} \\\\"
  print "  \hline"

  # header
  print "  exp & avg & max \\\\"
  print "  \hline \hline"

  for key, (avg_value, max_value) in avg_max_values:
    print "  " + key + " & " + float_format.format(avg_value) + " & " + float_format.format(max_value) + " \\\\"

  print "  \hline"
  print "\end{tabular}"




