# -*- coding: utf-8 -*-
import mit_gt
import kitti_gt
import level7_gt
import euroc_gt

def addProgramOptions(parser, required=False):

  group = parser.add_mutually_exclusive_group(required=required)

  group.add_argument('--mit', help='Ground truth file for the MIT dataset.')
  group.add_argument('--kitti', help='Ground truth file for the KITTI dataset.')
  group.add_argument('--level7', help='Ground truth file for the level7 dataset.')
  group.add_argument('--euroc', help='Ground truth file for the euroc dataset.')

def load( args ):

  if args.kitti:
    return kitti_gt.load( args.kitti )

  if args.level7:
    return level7_gt.load( args.level7 )

  if args.mit:
    return mit_gt.load( args.mit )
    
  if args.euroc:
    return euroc_gt.load( args.euroc )

  return None
