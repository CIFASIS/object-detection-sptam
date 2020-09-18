#!/bin/bash
# Usage:
# ./experiments/scripts/faster_rcnn_end2end.sh GPU NET DATASET [options args to {train,test}_net.py]
# DATASET is either pascal_voc or coco.
#
# Example
# ./experiments/scripts/faster_rcnn_end2end.sh 0 VGG_CNN_M_1024 pascal_voc \
#   --set EXP_DIR foobar RNG_SEED 42 TRAIN.SCALES "[400, 500, 600, 700]"

set -x
set -e

export PYTHONUNBUFFERED="True"

GPU_ID=$1
NET=$2
NET_lc=${NET,,}
DATASET=$3

array=( $@ )
len=${#array[@]}
EXTRA_ARGS=${array[@]:3:$len}
EXTRA_ARGS_SLUG=${EXTRA_ARGS// /_}

case $DATASET in
  pascal_voc)
    TRAIN_IMDB="voc_2007_trainval"
    TEST_IMDB="voc_2007_test"
    PT_DIR="pascal_voc"
    ITERS=70000
    ;;
  coco)
    # This is a very long and slow training schedule
    # You can probably use fewer iterations and reduce the
    # time to the LR drop (set in the solver to 350,000 iterations).
    TRAIN_IMDB="coco_2014_train"
    TEST_IMDB="coco_2014_minival"
    PT_DIR="coco"
    ITERS=490000
    ;;
  modelpose)
    TRAIN_IMDB="modelpose_train"
    TEST_IMDB="modelpose_voc.test"
    PT_DIR="modelpose"
    ITERS=20000
    ;;
  *)
    echo "No dataset given"
    exit
    ;;
esac

LOG="logs/faster_rcnn_endmod_${NET}_${EXTRA_ARGS_SLUG}.txt.`date +'%Y-%m-%d_%H-%M-%S'`"
exec &> >(tee -a "$LOG")
echo Logging output to "$LOG"

##set +x
##NET_FINAL=`grep -B 1 "done solving" ${LOG} | grep "Wrote snapshot" | awk '{print $4}'`
##set -x

#NET_FINAL="/state/partition1/javier/output/faster_rcnn_end2end/train/pose_cls_15k_40kcls_230kopose_p+c+r_iter_200000.caffemodel"
##NET_FINAL="/state/partition1/javier/output/faster_rcnn_end2end/train/pose_cls_15k50k_cont__iter_200000.caffemodel"
##NET_FINAL="/state/partition1/javier/output/faster_rcnn_end2end/coco_2014_train/coco_15kr_250ks__iter_5000.caffemodel"
## ultimo
##NET_FINAL="/state/partition1/javier/output/faster_rcnn_end2end/train/pose_cls_15k250k5k_dyp_20+_iter_30000.caffemodel"
NET_FINAL="/data/caffeModels/pose_coco_Allconst_iter16000.caffemodel"

##NET_FINAL="/state/partition1/javier/output/faster_rcnn_end2end/train/pose_cls_15k_onlypose__iter_40000.caffemodel"
##NET_FINAL="/state/partition1/javier/output/faster_rcnn_end2end/train/pose_cls_15k_15kcls_pose__iter_230000.caffemodel"


time ./tools/test_net.py --gpu ${GPU_ID} \
  --def ../models_trained/${PT_DIR}/${NET}/faster_rcnn_end2end/test.final.prototxt \
  --net ${NET_FINAL} \
  --imdb ${TEST_IMDB} \
  --cfg cfgs/faster_rcnn_posenet.yml \
  ${EXTRA_ARGS}

