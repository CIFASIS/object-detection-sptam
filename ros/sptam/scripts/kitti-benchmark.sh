#!/bin/bash
# Run KITTI_configuration

# S-PTAM directory
SPTAM_PATH=$HOME/catkin_ws/src/sptam

# S-PTAM build directory
SPTAM_BUILD_PATH=$SPTAM_PATH/src/standAlone/build

# executable stam-kitti path
SPTAM_KITTI=$SPTAM_BUILD_PATH/stam-KITTI

# S-PTAM configuration directory
SPTAM_CONFIGURATIONS_PATH=$SPTAM_PATH/configurationFiles

# S-PTAM configurations file
GFTT_BRIEF_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti.yaml
GFTT_BRISK_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_gftt_brisk.yaml
GFTT_LATCH_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_gftt_latch.yaml
FAST_BRIEF_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_fast_brief.yaml
FAST_BRISK_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_fast_brisk.yaml
FAST_LATCH_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_fast_latch.yaml
STAR_BRIEF_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_star_brief.yaml
STAR_BRISK_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_star_brisk.yaml
STAR_LATCH_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_star_latch.yaml
AGAST_BRIEF_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_agast_brief.yaml
AGAST_BRISK_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_agast_brisk.yaml
AGAST_LATCH_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_agast_latch.yaml
ORB_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_orb.yaml
SURF_CONFIGURATION_FILE=$SPTAM_CONFIGURATIONS_PATH/kitti_surf.yaml


# calibration file path
DATASET_PATH=$HOME/datasets/KITTI

# create output directory
OUTPUT_DIRECTORY=$(mktemp -d output_XXXXXXXXXXX)

# move to the output directory
cd $OUTPUT_DIRECTORY

# function to run sptam
run_sptam_kitti() {
  # set arguments to variables
  local sequence=$1
  local configuration_file=$2
  local calibration_file=$3
  $SPTAM_KITTI --images-source "dir" --timestamps $DATASET_PATH/${sequence}/times.txt --configuration ${configuration_file} --calibration ${SPTAM_CONFIGURATIONS_PATH}/${calibration_file} --left-images $DATASET_PATH/${sequence}/image_0/ --right-images $DATASET_PATH/${sequence}/image_1/
}

# run sptam over sequences
run_kitti_sequences() {

  # read configuration file argument
  local configuration_file=$1

  # read array argument
  declare -a sequences=("${!2}")

  echo "reading sequences:  ${sequences[@]}"

  ## define subset of sequences with respecto to each calibration - These sequences SHOULD NOT BE CHANGED #####

  sequences_00_to_02_13_to_21=(00 01 02 13 14 15 16 17 18 19 20 21)
  sequences_04_to_12=(04 05 06 07 08 09 10 11 12)
  sequence_03=03

  #############################################################################################################

  for i in ${sequences[@]}
  do
      echo "Executing sequence ${i}"

      # check if the element is in sequences: 00_to_02_13_to_21
      if [[ " ${sequences_00_to_02_13_to_21[@]} " =~ " ${i} " ]]
      then
        run_sptam_kitti ${i} ${configuration_file} kitti_cam_00_to_02_13_to_21.yaml
      fi

      # check if the element is in sequences: 03
      if [ ${i} -eq ${sequence_03} ]
      then
        run_sptam_kitti ${i} ${configuration_file} kitti_cam_03.yaml
      fi

      # check if the element is in sequences: 04_to_12
      if [[ " ${sequences_04_to_12[@]} " =~ " ${i} " ]]
      then
        run_sptam_kitti ${i} ${configuration_file} kitti_cam_04_to_12.yaml
      fi
  done
}

## sequences to run - CHOOSE THE SEQUENCES THAT YOU WANT TO RUN #############################################

# test sequences
test_sequences=(11 12 13 14 15 16 17 18 19 20 21)

# train sequences
training_sequences=(00 01 02 03 04 05 06 07 08 09 10)

#############################################################################################################


run_kitti_sequences ${GFTT_BRIEF_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${GFTT_BRISK_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${GFTT_LATCH_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${FAST_BRIEF_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${FAST_BRISK_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${FAST_LATCH_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${STAR_BRIEF_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${STAR_BRISK_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${STAR_LATCH_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${AGAST_BRIEF_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${AGAST_BRISK_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${AGAST_LATCH_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${ORB_CONFIGURATION_FILE} training_sequences[@]
#run_kitti_sequences ${SURF_CONFIGURATION_FILE} training_sequences[@]

