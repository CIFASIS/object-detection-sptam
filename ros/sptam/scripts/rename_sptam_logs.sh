#!/bin/bash


rename_log() {

  local sourceDir=$1
  local extension=log
  local pattern=source_left_images:

  cd $sourceDir

  local i=0
  for log_file in *.$extension; do

    # get the feature detector
    local detector=$(grep detector ${log_file} | cut -d' ' -f5)
    # pass to lowercase
    local detector=$(echo ${detector} | tr '[:upper:]' '[:lower:]')

    echo "detector: "${detector}

    # get the descriptor extractor
    local descriptor=$(grep descriptor ${log_file} | cut -d' ' -f5)
    # pass to lowercase
    local descriptor=$(echo ${descriptor} | tr '[:upper:]' '[:lower:]')

    echo "descriptor: "${descriptor}

    # get the sequence number
    local sequence=$(grep ${pattern} ${log_file} | cut -d/ -f 6)
    echo "sequence: "${sequence}

    # move file
    echo "trying to copy" ${log_file} "to" ${sequence}_${detector}_${descriptor}.${extension}
    cp $log_file ${sequence}_${detector}_${descriptor}.${extension}

  done
}

# call script which take the directory path as argument from command line
rename_log $1



