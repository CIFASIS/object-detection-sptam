#!/bin/bash


rename_log() {

  local sourceDir=$1
  local input_extension=log
  local output_extension=txt
  local source_images_pattern=source_left_images:
  local kitti_poses_pattern=BASE_LINK_POSE:


  cd $sourceDir

  local i=0
  for log_file in *.${input_extension}; do

    echo "Processing file: "${log_file}

    # get the sequence number
    local sequence=$(grep ${source_images_pattern} ${log_file} | cut -d/ -f 6)
    echo "sequence: "${sequence}


    # get the camera poses
    local poses=$(grep ${kitti_poses_pattern} ${log_file} | cut -d' ' -f3-14)
    #echo "poses: " "$poses"

    # create poses file
    echo "trying to copy Extracted poses to" ${sequence}.${output_extension}
    echo "$poses" > ${sequence}.${output_extension}

  done
}

# call script which take the directory path as argument from command line
rename_log $1



