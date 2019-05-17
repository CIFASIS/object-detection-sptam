# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

################################################################################
# PLOT HELPERS
################################################################################

def plotVsTime1(time, xs, title="", xlabel=None, ylabel=None):

  fig = plt.figure()
  ax = fig.add_subplot(111)

  p, = ax.plot(time, xs)

  ax.set_xlabel( xlabel )
  ax.set_ylabel( ylabel )

  plt.grid()

  fig.suptitle( title )

def plotVsTime3(time, xs, ys, zs, title=""):

  fig, (ax_x, ax_y, ax_z) = plt.subplots(3, sharex=True, sharey=True)

  px, = ax_x.plot(time, xs)
  py, = ax_y.plot(time, ys)
  pz, = ax_z.plot(time, zs)

  # Fine-tune figure; make subplots close to each other and hide x ticks for
  # all but bottom plot.
  #fig.subplots_adjust(hspace=0)
  plt.setp([a.get_xticklabels() for a in fig.axes[:-1]], visible=False)

  ax_z.set_xlabel("time (s)")

  ax_x.grid(True)
  ax_y.grid(True)
  ax_z.grid(True)

  fig.suptitle(title)

def plotVsTime4(time, xs, ys, zs, title=""):

  fig, (ax_x, ax_y, ax_z, ax_avg) = plt.subplots(4, sharex=True, sharey=True)

  px, = ax_x.plot(time, xs)
  py, = ax_y.plot(time, ys)
  pz, = ax_z.plot(time, zs)
  p_avg, = ax_avg.plot(time, np.linalg.norm(np.array([xs, ys, zs]), axis=0))

  # Fine-tune figure; make subplots close to each other and hide x ticks for
  # all but bottom plot.
  fig.subplots_adjust(hspace=0)
  plt.setp([a.get_xticklabels() for a in fig.axes[:-1]], visible=False)

  ax_avg.set_xlabel("time (s)")

  plt.grid()

  fig.suptitle(title)

  fig.tight_layout()

def plotPath3D(X, Y, Z, title):

  fig = plt.figure()

  ax = fig.add_subplot(111, projection='3d')

  ax.plot(X, Y, Z)

  #draw a point
  #~ ax.scatter([0],[0],[0],color="black",s=1)

  #draw cube
  #~ r = [-1, 1]
  #~ for s, e in combinations(np.array(list(product(r,r,r))), 2):
      #~ if np.sum(np.abs(s-e)) == r[1]-r[0]:
          #~ ax.plot3D(*zip(s,e), color="black")

  # simulate equal aspect ratio
  #~ """
  max_range = np.array([X.max()-X.min(), Y.max()-Y.min(), Z.max()-Z.min()]).max() / 2.0
  mean_x = X.mean()
  mean_y = Y.mean()
  mean_z = Z.mean()
  ax.set_xlim(mean_x - max_range, mean_x + max_range)
  ax.set_ylim(mean_y - max_range, mean_y + max_range)
  ax.set_zlim(mean_z - max_range, mean_z + max_range)
  #~ """
  # Set axis labels

  xLabel = ax.set_xlabel('x')
  yLabel = ax.set_ylabel('y')
  zLabel = ax.set_zlabel('z')

  ax.set_title(title)

def plotPaths3D(paths, labels, colors, title=None):
  """
  """

  assert( len(paths) == len(labels) )
  assert( len(paths) <= len(colors) )

  fig = plt.figure()

  ax = fig.add_subplot(111, projection='3d')

  for (X, Y, Z), label, color in zip(paths, labels, colors):
    ax.plot(X, Y, Z, label=label, color=color)

  #draw a point
  #~ ax.scatter([0],[0],[0],color="black",s=1)

  #draw cube
  #~ r = [-1, 1]
  #~ for s, e in combinations(np.array(list(product(r,r,r))), 2):
      #~ if np.sum(np.abs(s-e)) == r[1]-r[0]:
          #~ ax.plot3D(*zip(s,e), color="black")

  # simulate equal aspect ratio
  # assume first path is ground truth
  #"""
  max_range = np.array([paths[0][0].max()-paths[0][0].min(), paths[0][1].max()-paths[0][1].min(), paths[0][2].max()-paths[0][2].min()]).max() / 2.0
  mean_x = paths[0][0].mean()
  mean_y = paths[0][1].mean()
  mean_z = paths[0][2].mean()
  ax.set_xlim(mean_x - max_range, mean_x + max_range)
  ax.set_ylim(mean_y - max_range, mean_y + max_range)
  ax.set_zlim(mean_z - max_range, mean_z + max_range)
  #"""
  # Set axis labels

  xLabel = ax.set_xlabel('x (m)')
  yLabel = ax.set_ylabel('y (m)')
  zLabel = ax.set_zlabel('z (m)')

  handles, labels = ax.get_legend_handles_labels()
  ax.legend(handles, labels)

  if title:
    ax.set_title(title)

def plotPaths2D(paths, labels, colors, xlabel=None, ylabel=None, cloud_file=None, grid=None, save_filename=None):
  """
  """

  assert( len(paths) == len(labels) )
  assert( len(paths) <= len(colors) )

  # Plot the 2D trajectory (coordinate z = 0)
  fig = plt.figure()
  ax = fig.add_subplot(111)

  if ( cloud_file ):
    cloud = np.loadtxt( cloud_file )
    ax.scatter( cloud[:,0], cloud[:,1], s=1, color='0.1')

  for (X, Y), label, color in zip(paths, labels, colors):
    ax.plot(X, Y, label=label, color=color, linewidth=1.5)
    ax.plot(X[0], Y[0], marker='v',alpha=1, markersize=8, color=color)    

  # enable for KITTI 00 dataset
  #~ plt.xlim(-10, 50)
  #~ plt.ylim(0, 30)

  plt.gca().set_aspect('equal', adjustable='box')

  if xlabel:
    ax.set_xlabel( xlabel )

  if ylabel:
    ax.set_ylabel( ylabel )

  handles, labels = ax.get_legend_handles_labels()
  ax.legend(handles, labels, ncol=2, loc='center', bbox_to_anchor=(0.5, 1.2))

  if grid:
    ax.grid(True)
  
def plotLoops3D(paths, loops, title=None, time_unit=None):
  
  fig = plt.figure()

  ax = fig.add_subplot(111, projection='3d')

  for X, Y, Z, color in paths:
    ax.plot(X, Y, Z, color=color)
  
  for X, Y, Z, color in loops:
    ax.plot(X, Y, Z, color=color)

  # simulate equal aspect ratio
  # assume first path is ground truth
  #"""
  max_range = np.array([paths[0][0].max()-paths[0][0].min(), paths[0][1].max()-paths[0][1].min(), paths[0][2].max()-paths[0][2].min()]).max() / 2.0
  mean_x = paths[0][0].mean()
  mean_y = paths[0][1].mean()
  mean_z = paths[0][2].mean()
  ax.set_xlim(mean_x - max_range, mean_x + max_range)
  ax.set_ylim(mean_y - max_range, mean_y + max_range)
  ax.set_zlim(mean_z - max_range, mean_z + max_range)
  #"""
  # Set axis labels

  xLabel = ax.set_xlabel('x (m)')
  yLabel = ax.set_ylabel('y (m)')
  if time_unit:
    zLabel = ax.set_zlabel('time (' + time_unit + ')')
  else:
    zLabel = ax.set_zlabel('time (seconds)')
