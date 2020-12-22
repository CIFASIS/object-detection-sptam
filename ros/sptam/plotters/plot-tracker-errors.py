import math
import argparse
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

def filterByTask( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if words[2] == task:
      ret.append( np.array([float(words[0]), float(words[3])]) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )

def filterListByTask( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if words[2] == task:
      # append a tuple (timestamp, [...values...])
      ret.append( ( float(words[0]), np.array([ float(words[i]) for i in range(3, len(words)) ]) ) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return ret

if __name__ == "__main__":

  ####################################################################
  # Parse program options
  ####################################################################

  parser = argparse.ArgumentParser()

  parser.add_argument('filename', help='file containing the logged data')
  args = parser.parse_args()

  ####################################################################
  # Load data
  ####################################################################

  f = open(args.filename, 'r')
  lines = f.readlines()

  # tracker data

  error_outlier_count = filterByTask( lines, 'outliers:' )
  tracked_feat_count = filterByTask( lines, 'matched_feat_total:' )
  tracked_feat_left_count = filterByTask( lines, 'matched_feat_left:' )
  tracked_feat_right_count = filterByTask( lines, 'matched_feat_right:' )
  error_sigma_squared = filterByTask( lines, 'sigma_squared:' )
  error_squared_inliers = filterListByTask( lines, 'error_squared_inliers:' )
  error_squared_outliers = filterListByTask( lines, 'error_squared_outliers:' )

  #print(len(error_squared_outliers))
  #print(error_squared_outliers[0])

  ####################################################################
  # Plot sigma squared
  ####################################################################

  fig = plt.figure()
  ax = fig.add_subplot(111)

  timestamps = error_sigma_squared[:,0]
  ax.plot(timestamps, error_sigma_squared[:,1])

  #plt.xticks(np.arange( math.floor(min(timestamps)), math.ceil(max(timestamps)), 10.0 ))

  ax.grid(True)

  fig.suptitle("sigma squared")

  ####################################################################
  # Plot outlier percent
  ####################################################################
  """
  fig = plt.figure()
  ax = fig.add_subplot(111)

  ax.plot(error_outlier_count[:,0], error_outlier_count[:,1] / tracked_feat_count[:,1])

  ax.grid(True)

  fig.suptitle("outlier count")
  """
  ####################################################################
  # Plot outlier count
  ####################################################################

  fig = plt.figure()
  ax = fig.add_subplot(111)

  ax.plot(error_outlier_count[:,0], error_outlier_count[:,1], label='outliers')
  ax.plot(tracked_feat_count[:,0], tracked_feat_count[:,1], label='tracked')
  ax.plot(tracked_feat_left_count[:,0], tracked_feat_left_count[:,1], label='left')
  ax.plot(tracked_feat_right_count[:,0], tracked_feat_right_count[:,1], label='right')

  #plt.xticks(np.arange( math.floor(min(timestamps)), math.ceil(max(timestamps)), 10.0 ))

  ax.grid(True)

  handles, labels = ax.get_legend_handles_labels()
  ax.legend(handles, labels)

  fig.suptitle("outlier count")

  ####################################################################
  # Plot error boxplots
  ####################################################################
  
  fig = plt.figure()
  ax = fig.add_subplot(111)

  #ax.boxplot( [ errors for timestamp, errors in error_squared_inliers ] )

  #for timestamp, errors in error_squared_inliers:
  #  if 0<len(errors):
  #    ax.scatter(timestamp, min(errors), c='b')
  #    ax.scatter(timestamp, max(errors), c='b')

  #for timestamp, errors in error_squared_outliers:
  #  ax.scatter([timestamp for i in range(len(errors))], errors, c='r')

  for timestamp, errors in error_squared_outliers:
    if 0<len(errors):
      ax.scatter(timestamp, min(errors), c='r')
      ax.scatter(timestamp, max(errors), c='r')

  #ax.set_xlabel("time (s)")
  #ax.set_ylabel("euclidean error (m)")

  ax.grid(True)

  minimos = np.array( [ np.array([timestamp, min(errors)]) for timestamp, errors in error_squared_inliers if 0 < len(errors) ] )
  maximos = np.array( [ np.array([timestamp, max(errors)]) for timestamp, errors in error_squared_inliers if 0 < len(errors) ] )
  promedios = np.array( [ np.array([timestamp, np.mean(errors)]) for timestamp, errors in error_squared_inliers if 0 < len(errors) ] )

  ax.plot(minimos[:,0], minimos[:,1], c='b')
  ax.plot(maximos[:,0], maximos[:,1], c='b')
  ax.plot(promedios[:,0], promedios[:,1], c='g')

  #fig.suptitle("trayectory error")
  
  ####################################################################
  # Show all plots
  ####################################################################

  plt.show()
