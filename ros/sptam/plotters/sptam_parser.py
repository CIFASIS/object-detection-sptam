"""
S-PTAM output parser
"""

import re
import numpy as np

import tf.transformations
import mathHelpers as mh
import comparisonPlotter as comparador

implemented_datasets = ["kitti", "level7"]

################################################################################
# This functions parse data from the initial configuration lines
# printed at the beginning of the log files with prefix '#'.

def loadConfiguration( file_path ):

  file = open(file_path, 'r')
  lines = file.readlines()

  config = {}

  for line in lines:
    if line.lstrip().startswith('#'):

      # omit the '#' character and split the words
      line = line.strip()[1:].strip()

      words = line.split()

      # there should be two words: key and value with format ("key: value")
      if len(words)==2 and words[0][-1] == ':':
        config[ words[0][:-1] ] = words[1]
      else:
        print "skipping config line", line

  return config

def loadSequence( config ):

  # get the config field where the sequence number appears
  image_source_directory_path = config['source_left_images']

  # search for the sequence id (two digits) in the string
  re_digits = re.search('([0-9][0-9])', image_source_directory_path)

  if not re_digits:
    raise LookupError("Could not extract sequence id from file.")

  # I think search always returns the first match.
  # Only one group was specified in the RE.
  return re_digits.group(1)

################################################################################
# ground truth parsers

def find_nearest(array, value):
    return (np.abs(array-value)).argmin()

def filterNearest(timestamps, poses, query_timestamps):

  ret = []

  for t in query_timestamps:
    idx = find_nearest(timestamps, t)
    ret.append( poses[ idx ] )

  return ret

ground_truth_loaders = {
  "kitti": loadGroundTruthKITTI,
  "level7": loadGroundTruthLevel7,
}

def loadGroundTruth(dataset, filename):
  return ground_truth_loaders[ dataset ]( filename )

################################################################################
# Parse data from log files

def filterByTask( file, task ):

  task += ':'

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if 2<len(words) and words[2] == task:
      ret.append( np.array([float(words[0]), float(words[3])]) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")
  #~ else:
    #~ print(task, ":", len(ret))

  return np.array( ret )

def filterByTaskPoses( file, task ):

  ret = []

  for line in file:
    words = line.split()
    #if re.search( task, line ):
    if words[0] == task:
      ret.append( (np.array(words[1:])).astype(float) )

  if ( len( ret ) < 1 ):
    print("task", task, "was not measured")

  return np.array( ret )

def loadPoses( logfile ):
  
  f = open(logfile, 'r')
  lines = f.readlines()

  return filterByTaskPoses( lines, 'BASE_LINK_POSE:' )

class ExperimentData:

  def __init__(self, logfile, to_parse):

    f = open(logfile, 'r')
    lines = f.readlines()

    for task_label, task_id in to_parse.iteritems():
      setattr(self, task_id, filterByTask( lines, task_id ))

################################################################################
# pose data parsers

class PoseData:

  def __init__(self, experiment_poses, ground_truth_poses, label, align=False):

    assert( len(experiment_poses) == len(ground_truth_poses) )

    if align:
      experiment_poses = mh.alignToInitialPose( experiment_poses, ground_truth_poses[0] );
      ground_truth_poses = mh.alignToInitialPose( ground_truth_poses, experiment_poses[0] );

    #~ self.experiment_poses = experiment_poses
    #~ self.ground_truth_poses = ground_truth_poses

    print( experiment_poses[0] )
    print( ground_truth_poses[0] )

    absolute_error_poses = comparador.__computeErrorPoses__(experiment_poses, ground_truth_poses)
    self.absolute_translation_errors = comparador.__computeTranslationError__(absolute_error_poses)
    self.absolute_rotation_errors = comparador.__computeRotationError__(absolute_error_poses)

    #~ self.absolute_translation_errors = comparador.__getPositions__(absolute_error_poses)
    #~ self.absolute_rmse = computeRMSE( self.absolute_translation_errors )

    relative_error_poses = comparador.__computeRelativeErrorPoses__(experiment_poses, ground_truth_poses)
    self.relative_translation_errors = comparador.__computeTranslationError__(relative_error_poses)
    self.relative_rotation_errors = comparador.__computeRotationError__(relative_error_poses)

    #~ self.relative_translation_errors = comparador.__getPositions__(relative_error_poses)
    #~ self.relative_rmse = computeRMSE( self.relative_translation_errors )

    self.label = label

    assert( np.all( 0 < self.relative_translation_errors ) )

  def alignPosesTo(self, pos_ini, ori_ini):
    self.predicted_pos, self.predicted_ori = alignInitialFrame( self.predicted_pos, self.predicted_ori, pos_ini, ori_ini )
    self.adjusted_pos, self.adjusted_ori = alignInitialFrame( self.adjusted_pos, self.adjusted_ori, pos_ini, ori_ini )

def loadPoseDataKITTI(experiment_timestamps, experiment_poses, ground_truth_timestamps, ground_truth_poses, label):
  return PoseData(experiment_poses, ground_truth_poses, label)

def loadPoseDataLevel7(experiment_timestamps, experiment_poses, ground_truth_timestamps, ground_truth_poses, label):

  nearest_ground_truth_poses = filterNearest(ground_truth_timestamps, ground_truth_poses, experiment_timestamps)

  return PoseData(experiment_poses, nearest_ground_truth_poses, label)

pose_data_loaders = {
  "kitti": loadPoseDataKITTI,
  "level7": loadPoseDataLevel7,
}

def loadPoseData(dataset, experiment_timestamps, experiment_poses, ground_truth_timestamps, ground_truth_poses, label):
  return pose_data_loaders[ dataset ]( experiment_timestamps, experiment_poses, ground_truth_timestamps, ground_truth_poses, label )

################################################################################

def aggregateOverKey( experiments, key_function, data_function ):

  ret = {}

  for experiment_id, experiment_data in experiments.iteritems():
    for sequence_id, sequence_data in experiment_data.iteritems():

      key = key_function(experiment_id, sequence_id)

      if key not in ret:
        ret[key] = np.array([])

        data = data_function(sequence_data)
        aux = np.hstack((ret[key], data))
        assert( len(aux) == len(ret[key]) + len(data) )
        ret[key] = aux

  return ret

# Uses the list composition to make the key value pairs over a dictionary.
dict2list = lambda dic: [(k, v) for (k, v) in dic.iteritems()]

# Use the built in dictionary constructor to convert the list.
list2dict = lambda lis: dict(lis)

def mapDict(dictionary, function):

  return list2dict( map(lambda (k, v): (k, function(v)), dict2list(dictionary) ) )
