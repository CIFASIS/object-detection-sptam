S-PTAM is a Stereo SLAM system able to compute the camera trajectory in real-time. It heavily exploits the parallel nature of the SLAM problem, separating the time-constrained pose estimation from less pressing matters such as map building and refinement tasks. On the other hand, the stereo setting allows to reconstruct a metric 3D map for each frame of stereo images, improving the accuracy of the mapping process with respect to monocular SLAM and avoiding the well-known bootstrapping problem. Also, the real scale of the environment is an essential feature for robots which have to interact with their surrounding workspace.

You should have received this sptam version along with object-detection-sptam (https://github.com/CIFASIS/object-detection-sptam).
See the original sptam library at: https://github.com/CIFASIS/sptam/tree/s-ptam-iros2015
All files included in this sptam version are GPLv3 license.

