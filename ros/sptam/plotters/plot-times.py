#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    if 2<len(words) and words[2] == task:
      ret.append( np.array([float(words[0]), float(words[3])]) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )

def plotTask( ax, file, task ):

  data = filterByTask( file, task )

  if ( len(data) < 1 ):
    print("no entries found for task '", task, "'")
    return

  ax.plot(data[:,0], data[:,1], label=task)

def plotStack( ax, stack, total, labels ):

  # search for minimum length array
  lim = min( min( [ len(data) for data in stack ] ), len(total) )

  # truncate arrays to have the same length
  stack = [ data[:lim,1] for data in stack ]

  # plot the stacked functions
  sp = ax.stackplot( total[:lim,0], stack )

  # plot the total function
  # TODO label not working
  ax.plot(total[:lim,0], total[:lim,1], label='total')

  # we need to use a proxy artist to handle legends in a stackplot
  proxy = [mpl.patches.Rectangle((0,0), 0,0, facecolor=pol.get_facecolor()[0]) for pol in sp]
  ax.legend(proxy, labels)

  return lim

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

  data_extract_features = filterByTask( lines, 'extraction:' )
  data_frustum_filter = filterByTask( lines, 'frustum:' )
  data_find_matches = filterByTask( lines, 'find_matches:' )
#  data_lock_update_descriptors = filterByTask( lines, 'lock_update_descriptors:' )
  data_update_descriptors = filterByTask( lines, 'update_descriptors:' )
  data_tracking_refine = filterByTask( lines, 'tracking_refine:' )
  data_keyframe_selection_strategy = filterByTask( lines, 'keyframe_selection_strategy:')
  data_triangulate_points = filterByTask( lines, 'triangulate_points:' )
  data_add_points = filterByTask( lines, 'add_points:' )
  data_add_keyframe = filterByTask( lines, 'add_keyframe:' )
  data_add_meas = filterByTask( lines, 'add_meas:' )
  data_lock_add_meas = filterByTask( lines, 'lock_add_meas:' )
  data_lock_add_points = filterByTask( lines, 'lock_add_points:' )
  data_lock_frustum = filterByTask( lines, 'lock_frustum:' )
  data_tracking_total = filterByTask( lines, 'trackingtotal:' )

  data_visible_points = filterByTask( lines, 'visiblePoints:' )
  data_created_new_points = filterByTask( lines, 'created_new_points:' )
  data_matched_points = filterByTask( lines, 'matched_feat_total:' )

  # mapper data

  data_queue_size = filterByTask( lines, 'queueSize:' )
  data_refind_single = filterByTask( lines, 'refind_newly_made:' )
  data_total_kfs = filterByTask( lines, 'totalKeyFrames:' )
  data_total_points = filterByTask( lines, 'totalPoints:' )

  data_ba_local = filterByTask( lines, 'local:' )
  data_ba_local_select = filterByTask( lines, 'local_select:' )
  data_ba_local_load = filterByTask( lines, 'local_load:' )
  data_ba_local_adjust = filterByTask( lines, 'local_adjust:' )
  data_ba_local_save_points = filterByTask( lines, 'local_save_points:' )
  data_ba_local_save_cameras = filterByTask( lines, 'local_save_cameras:' )
  data_ba_local_handle_bad = filterByTask( lines, 'local_handle_bad:' )

  data_refind_new = filterByTask( lines, 'refind_newly_made:' )
  data_remove_bad_points_lock = filterByTask( lines, 'remove_bad_points_lock:' )

  data_ba_global = filterByTask( lines, 'global:' )
  data_ba_global_load = filterByTask( lines, 'global_load:' )
  data_ba_global_adjust = filterByTask( lines, 'global_adjust:' )
  data_ba_global_save_points = filterByTask( lines, 'global_save_points:' )
  data_ba_global_save_cameras = filterByTask( lines, 'global_save_cameras:' )
  data_ba_global_handle_bad = filterByTask( lines, 'global_handle_bad:' )

  data_handle_bad = filterByTask( lines, 'handleBadPoints:' )
  data_ba_total = filterByTask( lines, 'totalba:' )

  # Visualization data (PCL timing)

  # data_points_visualization = filterByTask( lines, 'points_visualization:' )


  ####################################################################
  # Plot all tracker lines
  ####################################################################

  fig, (ax_time, ax_points) = plt.subplots(2, sharex=True)

  fig.suptitle("All tracker operations")

  ax_time.plot(data_extract_features[:,0], data_extract_features[:,1], label='feature extraction')
  ax_time.plot(data_frustum_filter[:,0], data_frustum_filter[:,1], label='frustum filter')
  ax_time.plot(data_find_matches[:,0], data_find_matches[:,1], label='find matches')
#  ax_time.plot(data_lock_update_descriptors[:,0], data_lock_update_descriptors[:,1], label='lock update descriptors')
  ax_time.plot(data_update_descriptors[:,0], data_update_descriptors[:,1], label='update descriptors')
  ax_time.plot(data_tracking_refine[:,0], data_tracking_refine[:,1], label='tracking refinement')
  ax_time.plot(data_keyframe_selection_strategy[:,0], data_keyframe_selection_strategy[:,1], label='selection strategy')
  ax_time.plot(data_triangulate_points[:,0], data_triangulate_points[:,1], label='triangulate points')
  ax_time.plot(data_add_points[:,0], data_add_points[:,1], label='add points')
#  ax_time.plot(data_add_keyframe[:,0], data_add_keyframe[:,1], label='add keyframe')
  ax_time.plot(data_add_meas[:,0], data_add_meas[:,1], label='add meas')
  ax_time.plot(data_lock_add_meas[:,0], data_lock_add_meas[:,1], label='lock add meas')
  ax_time.plot(data_lock_add_points[:,0], data_lock_add_points[:,1], label='lock add points')
  ax_time.plot(data_lock_frustum[:,0], data_lock_frustum[:,1], label='lock frustum')
  ax_time.plot(data_tracking_total[:,0], data_tracking_total[:,1], label='tracking total')
  ax_time.set_ylabel("time consumed (s)")

  ax_points.plot(data_visible_points[:,0], data_visible_points[:,1], label='visible points')
  ax_points.plot(data_matched_points[:,0], data_matched_points[:,1], label='matched points')
  ax_points.plot(data_created_new_points[:,0], data_created_new_points[:,1], label='created points')
  ax_points.set_ylabel("number of points")

  ax_points.set_xlabel("time elapsed (s)")

  ax_time.grid(True)
  ax_points.grid(True)

  handles, labels = ax_time.get_legend_handles_labels()
  ax_time.legend(handles, labels)

  handles, labels = ax_points.get_legend_handles_labels()
  ax_points.legend(handles, labels)

  ####################################################################
  # Plot total tracking as function of visible points
  ####################################################################

  # the arrays may have different length if the process was terminated
  # before finishing the sequence.
  """
  lim = min(len(data_tracking_total), len(data_visible_points))

  fig = plt.figure()
  ax = fig.add_subplot(111)

  ax.scatter(data_visible_points[:lim,1], data_tracking_total[:lim,1])

  ax.set_xlabel("visible points")
  ax.set_ylabel("total tracking time (s)")
  """

  ####################################################################
  # Stacked tracker plot
  ####################################################################
# HAY UN BUG CON ESTE PLOTTING PORQUE NO HAY QUE USAR EN EL EJE X EL TIMPO SINO QUE EL NUMERO DE FRAME PARA QUE TODOS PERTENEZCAN AL MISMO FRAME
  fig, (ax_time, ax_points) = plt.subplots(2, sharex=True)

  fig.suptitle("Stacked tracker operations")

  stack = [
    data_extract_features,
    data_frustum_filter,
    data_find_matches,
    data_update_descriptors,
    data_tracking_refine,
    data_keyframe_selection_strategy,
    data_triangulate_points,
    data_add_points,
    data_add_meas,
    data_lock_add_points,
    data_lock_add_meas,
    data_lock_frustum,
  ]

  lim = plotStack( ax_time, stack, data_tracking_total, ('feature extraction', 'frustum filter', 'find matches', 'update descriptors', 'tracking refinement', 'selection strategy', 'triangulate points', 'add points', 'add meas', 'lock add points', 'lock add meas', 'lock frustum'))

  ax_points.plot(data_visible_points[:,0], data_visible_points[:,1], label='visible points')
  ax_points.plot(data_matched_points[:,0], data_matched_points[:,1], label='matched points')
  ax_points.plot(data_created_new_points[:,0], data_created_new_points[:,1], label='created points')

  ax_points.set_xlabel("time elapsed (s)")
  ax_points.set_ylabel("number of points points")
  ax_time.set_ylabel("time consumed (s)")

  ax_time.grid(True)
  ax_points.grid(True)

  handles, labels = ax_points.get_legend_handles_labels()
  ax_points.legend(handles, labels)

  ####################################################################
  # Percentage plot
  ####################################################################

  """
  fig = plt.figure()
  ax = fig.add_subplot(111)

  lim = min(
    len(data_extract_features),
    len(data_frustum_filter),
    len(data_find_matches),
    len(data_tracking_refine),
    len(data_match_global),
    len(data_tracking_total),
  )

  data_extract_features_n = np.divide(data_extract_features[:lim,1], data_tracking_total[:lim,1])
  data_frustum_filter_n = np.divide(data_frustum_filter[:lim,1], data_tracking_total[:lim,1])
  data_find_matches_n = np.divide(data_find_matches[:lim,1], data_tracking_total[:lim,1])
  data_tracking_refine_n = np.divide(data_tracking_refine[:lim,1], data_tracking_total[:lim,1])
  data_match_global_n = np.divide(data_match_global[:lim,1], data_tracking_total[:lim,1])

  sp = ax.stackplot( data_tracking_total[:lim,0],
    data_extract_features_n,
    data_frustum_filter_n,
    data_find_matches_n,
    data_tracking_refine_n,
    data_match_global_n,
  )

  ax.set_xlabel("time elapsed (s)")
  ax.set_ylabel("percent of total tracking")

  ax.grid(True)
  ax.grid(True)

  # we need to use a proxy artist to handle legends in a stackplot
  proxy = [mpl.patches.Rectangle((0,0), 0,0, facecolor=pol.get_facecolor()[0]) for pol in sp]
  ax.legend(proxy, ('feature extraction', 'frustum filter', 'find matches', 'tracking refinement'))
  """

  ####################################################################
  # Plot all mapper lines
  ####################################################################

  fig, (ax_time, ax_points) = plt.subplots(2, sharex=True)

  fig.suptitle("all mapper operations")

  #fig.suptitle("time consumed per task")

  ax_time.plot(data_refind_single[:,0], data_refind_single[:,1], label='refind in single')
  ax_time.plot(data_ba_local[:,0], data_ba_local[:,1], label='local BA')
  ax_time.plot(data_refind_new[:,0], data_refind_new[:,1], label='refind in new')
#  ax_time.plot(data_ba_global[:,0], data_ba_global[:,1], label='global BA')
  ax_time.plot(data_handle_bad[:,0], data_handle_bad[:,1], label='handle bad points')
  ax_time.plot(data_remove_bad_points_lock[:,0], data_remove_bad_points_lock[:,1], label='remove bad points lock')
  ax_time.plot(data_ba_total[:,0], data_ba_total[:,1], label='total BA')
  ax_time.set_ylabel("time consumed (s)")

  #ax_points.plot(data_total_kfs[:,0], data_total_kfs[:,1], label='total keyframes')
  #ax_points.plot(data_total_points[:,0], data_total_points[:,1], label='total points')
  ax_points.plot(data_queue_size[:,0], data_queue_size[:,1], label='queue size')

  ax_points.set_xlabel("time elapsed (s)")

  ax_time.grid(True)
  ax_points.grid(True)

  handles, labels = ax_time.get_legend_handles_labels()
  ax_time.legend(handles, labels)

  handles, labels = ax_points.get_legend_handles_labels()
  ax_points.legend(handles, labels)

  ####################################################################
  # Stacked local BA plot
  ####################################################################

  fig, (ax_time, ax_points) = plt.subplots(2, sharex=True)

  fig.suptitle("Stacked local BA operations")

  stack = [
    data_ba_local_select,
    data_ba_local_load,
    data_ba_local_adjust,
    data_ba_local_save_points,
    data_ba_local_save_cameras,
    data_ba_local_handle_bad,
  ]

  lim = plotStack( ax_time, stack, data_ba_local, ('select', 'load', 'adjust', 'save points', 'save views', 'handle bad') )

  ax_points.plot(data_ba_local[:lim,0], data_visible_points[:lim,1], label='visible points')

  ax_points.set_xlabel("time elapsed (s)")
  ax_points.set_ylabel("visible points")
  ax_time.set_ylabel("time consumed (s)")

  ax_time.grid(True)
  ax_points.grid(True)

  ####################################################################
  # Stacked global BA plot
  ####################################################################

#  fig, (ax_time, ax_points) = plt.subplots(2, sharex=True)

#  fig.suptitle("Stacked global BA operations")

#  stack = [
#    data_ba_global_load,
#    data_ba_global_adjust,
#    data_ba_global_save_points,
#    data_ba_global_save_cameras,
#    data_ba_global_handle_bad,
#  ]

#  lim = plotStack( ax_time, stack, data_ba_global, ('load', 'adjust', 'save points', 'save views', 'handle bad') )

#  ax_points.plot(data_ba_global[:lim,0], data_total_points[:lim,1], label='visible points')

#  ax_points.set_xlabel("time elapsed (s)")
#  ax_points.set_ylabel("visible points")
#  ax_time.set_ylabel("time consumed (s)")

#  ax_time.grid(True)
#  ax_points.grid(True)

  ####################################################################
  # Plot Visualization Stuffs (PCL timing)
  ####################################################################


#  fig = plt.figure()
#  ax_time = fig.add_subplot(111)
#  ax_time.set_title("Visualizarion Operations")

#  ax_time.plot(data_points_visualization[:,0], data_points_visualization[:,1], label='Points Visualization')
#  ax_time.set_ylabel("time consumed (s)")
#  ax_time.set_xlabel("time elapsed (s)")

#  ax_time.grid(True)
#  ax_points.grid(True)

#  handles, labels = ax_time.get_legend_handles_labels()
#  ax_time.legend(handles, labels)

  ####################################################################
  # Plot Measurements and Points Stuffs
  ####################################################################

#  points_measurement_number = filterByTask( lines, 'MeasurementCount:' )

#  fig = plt.figure()
#  ax = fig.add_subplot(111)

#  ax.plot(range(0, len( points_measurement_number )), points_measurement_number[:,1], '.')

#  ax.set_ylabel("Measurements count")
#  ax.set_xlabel("points")

#  plt.grid()

#  fig.suptitle("Measurements count")

  ####################################################################
  # Print tracking main steps times
  ####################################################################

  mean_extract_features_time = np.mean(data_extract_features[:,1])
  mean_frustum_filter_time = np.mean(data_frustum_filter[:,1])
  mean_find_matches_time = np.mean(data_find_matches[:,1])
  mean_tracking_refine_time = np.mean(data_tracking_refine[:,1])
  mean_add_keyframe_time = np.mean(data_add_keyframe[:,1])
  mean_total_tracking_time = np.mean(data_tracking_total[:,1])

  print ("mean_extract_features_time:", mean_extract_features_time)
  print ("mean_frustum_filter_time:", mean_frustum_filter_time)
  print ("mean_find_matches_time:", mean_find_matches_time)
  print ("mean_tracking_refine_time:", mean_tracking_refine_time)
  print ("mean_add_keyframe_time:", mean_add_keyframe_time)
  print ("mean_total_tracking_time_sum:", mean_extract_features_time + mean_frustum_filter_time + mean_find_matches_time + mean_tracking_refine_time + mean_add_keyframe_time)
  print ("mean_total_tracking_time_measured:", mean_total_tracking_time)


  ####################################################################
  # Show plots
  ####################################################################

  plt.show()
