#!/usr/bin/env python

# --------------------------------------------------------
# Faster R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

"""
Demo script showing detections in sample images.

See README.md for installation instructions before running.
"""
import _init_paths

from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms

from utils.timer import Timer
## esto es por si da este error por no tener X
## https://stackoverflow.com/questions/37604289/tkinter-tclerror-no-display-name-and-no-display-environment-variable 
import matplotlib
matplotlib.use('Agg')
####
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys, cv2
import argparse

import rospy
from dl_node.msg import DetectionWithPose
from dl_node.msg import DetectionWithPoseList

import math

CLASSES = ('__background__',
           'bathtub', 'bed', 'chair', 'desk',
           'dresser', 'monitor', 'night_stand', 'sofa', 'table',
           'toilet')

##NETS = {'vgg16':('VGG16','pose_cls_15k_40kcls_pose__iter_230000.caffemodel')}
NETS= {'vgg16':('VGG16',"pose_cls_15k_40kcls_230kopose_p+c+r_iter_200000.caffemodel")} ##best
##NETS ={'vgg16':('VGG16','pose_cls_15k_0kcls_pose_170000_resume_iter_60000.caffemodel')}
##NETS = {'vgg16': ('VGG16','pose_cls_2_onlypose__iter_130000.caffemodel')} 
##NETS = {'vgg16' : ('VGG16','pose_cls_15k_15kcls_pose__iter_230000.caffemodel')}

FOLDER = 'abspath'

                 

#def parse_args():
#    """Parse input arguments."""
#    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
#    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
#                        default=0, type=int)
#    parser.add_argument('--cpu', dest='cpu_mode',
#                        help='Use CPU mode (overrides --gpu)',
#                        action='store_true')
#    parser.add_argument('--net', dest='demo_net', help='Network to use [vgg16]',
#                        choices=NETS.keys(), default='vgg16')

#    args = parser.parse_args()

#    return args

##WARM UP AND LOAD NETWORK   
    
cfg.TEST.HAS_RPN = True  # Use RPN for proposals

#args = parse_args()
demo_net = 'vgg16'
cpu_mode = False 
gpu_id = 0
#

prototxt = os.path.join(cfg.MODELS_DIR, 'modelpose', NETS[demo_net][0],
                            'faster_rcnn_end2end', 'test.final.prototxt')
if FOLDER == 'data':
   caffemodel = os.path.join(cfg.DATA_DIR, 'faster_rcnn_models',
                                 NETS[demo_net][1])
elif FOLDER == 'partition1':
   caffemodel = os.path.join('/home/taihu/output/faster_rcnn_end2end/train',NETS[demo_net][1])

elif FOLDER == 'abspath':

   caffemodel = '/data/caffeModels/pose_coco_Allconst_iter16000.caffemodel'

else:
   caffemodel = os.path.join(cfg.ROOT_DIR, 'output','faster_rcnn_end2end','train',
                                 NETS[demo_net][1])



if not os.path.isfile(caffemodel):
  raise IOError(('{:s} not found.\nDid you run ./data/script/'
                   'fetch_faster_rcnn_models.sh?').format(caffemodel))

if cpu_mode:
    caffe.set_mode_cpu()
else:
    caffe.set_mode_gpu()
    caffe.set_device(gpu_id)
    cfg.GPU_ID = gpu_id
NET = caffe.Net(prototxt, caffemodel, caffe.TEST)

print '\n\nLoaded network {:s}'.format(caffemodel)

# Warmup on a dummy image
dummy_im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
for i in xrange(2):
    _, _= im_detect(NET, dummy_im)

##if __name__ == '__main__':

def demo(im):
    caffe.set_mode_gpu()
    caffe.set_device(0)
    cfg.GPU_ID = 0
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals


    """Detect object classes in an image using pre-computed object proposals."""
    
    # Detect all object classes and regress object bounds
    
    ## Uncomment for time info
    #timer = Timer()
    #timer.tic()
    scores, boxes = im_detect(NET, im, None)
    #timer.toc()
    #print ('Detection took {:.3f}s for '
    #       '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    ###  
    #im_show = im[:, :, (2, 1, 0)]
    #fig, ax = plt.subplots(figsize=(12, 12))
    #ax.imshow(im_show, aspect='equal')
    ###

    # Visualize detections for each class
    CONF_THRESH = 0.8
    #NMS default = 0.3
    NMS_THRESH = 0.3
    CV_AA = 16
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1 # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        
        ##vis_detections(im, cls, dets, thresh=CONF_THRESH)
        thresh=CONF_THRESH
        
        ###"""Draw detected bounding boxes."""
        inds = np.where(dets[:, -1] >= thresh)[0]
        ##if len(inds) == 0:
        ##  return
        
        for i in inds:
           ##Prueba
           x1, y1, x2, y2 = map(int, dets[i, :4])
           cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2, CV_AA)
           ret, baseline = cv2.getTextSize(
               CLASSES[cls_ind], cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)
           cv2.rectangle(im, (x1, y2 - ret[1] - baseline),
                         (x1 + ret[0], y2), (0, 0, 255), -1)
           cv2.putText(im, CLASSES[cls_ind], (x1, y2 - baseline),
              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, CV_AA)
           
           ##bbox = dets[i, :4]
           ##score = dets[i, -1]
           #ax.add_patch(
           #    plt.Rectangle((bbox[0], bbox[1]),
           #                  bbox[2] - bbox[0],
           #                  bbox[3] - bbox[1], fill=False,
           #                  edgecolor='red', linewidth=3.5)
           #    )
           #ax.text(bbox[0], bbox[1] - 2,
           #        '{:s} {:.3f}'.format(class_name, score),
           #        bbox=dict(facecolor='blue', alpha=0.5),
           #        fontsize=14, color='white')
    #plt.axis('off')
    #plt.tight_layout()
    #plt.draw()
    #plt.show()
    #print('show ok')
    return im

def getDetections(im,show):
    
    caffe.set_mode_gpu()
    caffe.set_device(0)
    cfg.GPU_ID = 0
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals 
    cfg.TEST.HAS_POSE = True ##POSE


    """Detect object classes in an image using pre-computed object proposals."""

    # Detect all object classes and regress object bounds

    ## Uncomment for time info
    #timer = Timer()
    #timer.tic()
    scores, boxes,angle_deltas,angle_scores,size_deltas = im_detect(NET, im)
    #timer.toc()
    #print ('Detection took {:.3f}s for '
    #       '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # filter detections for each class
    CONF_THRESH = 0.85
    #NMS default = 0.3
    NMS_THRESH = 0.33
    CV_AA = 16

    detections = ()

    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1 # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        
        thcls_scores = angle_scores[keep,  8*cls_ind : 8*(cls_ind+1)]
        thcls_deltas = angle_deltas[keep, 16*cls_ind :16*(cls_ind+1)]
        
        sizecls_deltas = size_deltas[keep, 3*cls_ind : 3*(cls_ind+1)]
        mean = cfg.SIZE_MEANS[cls_ind]
        
        ##vis_detections(im, cls, dets, thresh=CONF_THRESH)
        thresh=CONF_THRESH

        ###"""Draw detected bounding boxes."""
        inds = np.where(dets[:, -1] >= thresh)[0]
        ##if len(inds) == 0:
        ##  return
        
        for i in inds:
           ##Prueba
           x1, y1, x2, y2 = map(int, dets[i, :4])
           detect = DetectionWithPose()
           detect.cls = cls_ind
           detect.x1 = x1
           detect.y1 = y1
           detect.x2 = x2
           detect.y2 = y2
           
           angle_bin = np.argmax(thcls_scores[i,:])
           detect.yaw =  angle_bin  * (2*math.pi / 8.0) + math.atan2(thcls_deltas[i,angle_bin*2],thcls_deltas[i,angle_bin*2 + 1])
           detect.dimX = sizecls_deltas[i,0] + mean[0]
           detect.dimY = sizecls_deltas[i,1] + mean[1]
           detect.dimZ = sizecls_deltas[i,2] + mean[2]
           
           detections = detections + (detect,)
        
           if show == 1 :
              cv2.rectangle(im, (x1, y1), (x2, y2), (0, 0, 255), 2, CV_AA)
              ret, baseline = cv2.getTextSize(
                  CLASSES[cls_ind], cv2.FONT_HERSHEY_SIMPLEX, 0.8, 1)
              cv2.rectangle(im, (x1, y2 - ret[1] - baseline),
                            (x1 + ret[0], y2), (0, 0, 255), -1)
              cv2.putText(im, CLASSES[cls_ind]+str(dets[i,-1])+" "+str(detect.yaw), (x1, y2 - baseline),
              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, CV_AA)

           
    
    return detections,im        

