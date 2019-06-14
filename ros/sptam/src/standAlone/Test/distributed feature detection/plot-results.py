import math
import argparse
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

def filterByMethod( file, method ):

  ret = []

  for line in file:
    words = line.split()
    if words[0] == method:
      ret.append( np.array([int(words[1]), float(words[2])]) )

  if ( len( ret ) < 1 ):
    print("method", method, "not measured")

  return np.array( ret )

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

  data_good = filterByMethod( lines, 'goodFeat' )
  data_star = filterByMethod( lines, 'star/brief' )
  data_fast = filterByMethod( lines, 'fast/brief' )
  data_orb  = filterByMethod( lines, 'orb/orb' )

  ####################################################################
  # plot data
  ####################################################################

  fig, (ax_time, ax_points) = plt.subplots(2, sharex=True)

  fig.suptitle("Feature extractor comparison")

  ax_time.plot(data_good[:,1], label='Shi - Tomasi')
  ax_time.plot(data_star[:,1], label='Star / Brief')
  ax_time.plot(data_fast[:,1], label='Fast / Brief')
  ax_time.plot( data_orb[:,1], label='Orb')
  ax_time.set_ylabel("time consumed (s)")


  ax_points.plot(data_good[:,0], label='Shi - Tomasi')
  ax_points.plot(data_star[:,0], label='Star / Brief')
  ax_points.plot(data_fast[:,0], label='Fast / Brief')
  ax_points.plot( data_orb[:,0], label='Orb')
  ax_points.set_ylabel("extracted points")

  ax_points.set_xlabel("frame")

  ax_time.grid(True)
  ax_points.grid(True)

  handles, labels = ax_time.get_legend_handles_labels()
  ax_time.legend(handles, labels)

  handles, labels = ax_points.get_legend_handles_labels()
  ax_points.legend(handles, labels)

  plt.show()
