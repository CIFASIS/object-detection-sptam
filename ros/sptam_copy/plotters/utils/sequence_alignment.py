# -*- coding: utf-8 -*-

"""
Utilities for aligning sptam sequences, usually against ground truths.
Mostly used by ground truth parsers.
"""

# returns t1 < t2 
def __lessThan__(t1, t2):

  return (t1 - t2).to_sec() < 0

# returns t1 <= t2 
def __lessThanOrEqual__(t1, t2):

  return (t1 - t2).to_sec() <= 0

## clip time offsets from beginning and end of sequences

def getTimestampOffsetStartingIndex(timestamps, start_offset):
  """
  @return
    an index 'i' such that timestamps[i] is the minimum timestamp that satisfies timestamps[0] + start_offset (s) <= timestamps[i]
  """

  t0 = timestamps[0] + rospy.Time.from_sec( start_offset )

  if __lessThan__(timestamps[-1], t0):
    raise RuntimeError("timestamps timespan is shorter than requested offset")

  # get first array index where the elapsed time is greater than or equal to start_offset.
  # min(i for i, t in enumerate( times ) if start_offset <= (t - t0).to_sec())
  return next(i for i, t in enumerate( timestamps ) if __lessThanOrEqual__(t0, t))

def getTimestampOffsetEndingLimit(timestamps, end_offset):
  """
  @return
    an index 'i' such that timestamps[i-1] is the maximum timestamp that satisfies timestamps[i-1] < timestamps[-1] - end_offset (s)
  """

  tn = timestamps[-1] - rospy.Time.from_sec( end_offset )

  if __lessThan__(tn, timestamps[0]):
    raise RuntimeError("timestamps timespan is shorter than requested offset")

  # get first array index from which on the remaining time is greater than end_offset.
  # max(i for i, t in enumerate( times ) if (t - t0).to_sec() <= end_offset)
  return len(timestamps) - next(i for i, t in enumerate( reversed( timestamps ) ) if __lessThan__(t, tn))

## clip a query sequence to be contained in a reference sequence

def getTimestampSubsetStartingIndex(reference_timestamps, query_timestamps):
  """
  @param reference_timestamps
    iterable of sorted rospy.Time objects.

  @param query_timestamps
    iterable of sorted rospy.Time objects.

  @return
    an index 'i' such that query_timestamps[i] is the minimum queried timestamp that satisfies reference_timestamps[0] <= query_timestamps[i].
  """

  t0 = reference_timestamps[0]

  if __lessThan__(query_timestamps[-1], t0):
    raise RuntimeError("queried timestamps happen much sooner than reference timestamps")

  return next(i for i, t in enumerate( query_timestamps ) if __lessThanOrEqual__(t0, t))

def getTimestampSubsetEndingLimit(reference_timestamps, query_timestamps):
  """
  @param reference_timestamps
    iterable of sorted rospy.Time objects.

  @param query_timestamps
    iterable of sorted rospy.Time objects.

  @return
    an index 'i' such that query_timestamps[i-1] is the maximum queried timestamp that satisfies query_timestamps[i-1] < reference_timestamps[-1].
  """

  tn = reference_timestamps[-1]

  if __lessThan__(tn, query_timestamps[0]):
    raise RuntimeError("queried timestamps happen much later than reference timestamps")

  return len(query_timestamps) - next(i for i, t in enumerate( reversed( query_timestamps ) ) if __lessThanOrEqual__(t, tn))

def getTimestampSubset(reference_timestamps, query_timestamps):
  """
  @param reference_timestamps
    iterable of sorted rospy.Time objects. This is the containing set.

  @param query_timestamps
    iterable of sorted rospy.Time objects. This is the set that wants to be clipped for inclusion in the containing set.

  @return
    pair of indexes (a, b) such that query_timestamps[a:b] is the maximal set contained in reference_timestamps.
  """

  start_idx = getTimestampSubsetStartingIndex(reference_timestamps, query_timestamps)
  end_idx = getTimestampSubsetEndingLimit(reference_timestamps, query_timestamps)

  return start_idx, end_idx

## find corresponding timestamps in two sequences

# bisect does not take a custom comparison function,
# and the rospy.Time comparison does not seem to work,
# so we use a custom comparison binary search
def find_nearest_timestamp(reference_timestamps, query_timestamp):

  if __lessThan__(query_timestamp, reference_timestamps[0]):
    print("WARNING: sptam querying timestamps before GT start")
    return 0

  if __lessThan__(reference_timestamps[-1], query_timestamp):
    print("WARNING: sptam querying timestamps after GT end")
    return -1

  if 1 == len( reference_timestamps ):
    return 0

  pivot_idx = len( reference_timestamps ) / 2

  smaller_half = reference_timestamps[:pivot_idx]
  bigger_half = reference_timestamps[pivot_idx:]

  if __lessThan__(query_timestamp, smaller_half[-1]):
    return find_nearest_timestamp(smaller_half, query_timestamp)

  if __lessThan__(bigger_half[0], query_timestamp):
    return pivot_idx + find_nearest_timestamp(bigger_half, query_timestamp)

  # if we arrive here, query_timestamp is between both array limits
  if (query_timestamp - smaller_half[-1]).to_sec() <= (bigger_half[0] - query_timestamp).to_sec():
    return pivot_idx - 1
  # else
  return pivot_idx

def find_nearest_timestamps(reference_timestamps, query_timestamps):

  return [ find_nearest_timestamp(reference_timestamps, t) for t in query_timestamps ]
