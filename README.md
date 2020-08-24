*object-detection-sptam* is a SLAM system for stereo cameras which builds a map of objects in a scene. The system is based on the SLAM method S-PTAM and an object detection module. The object detection module uses Deep Learning to perform online detection and provide the 3d pose estimations of objects present in an input image, while S-PTAM estimates the camera pose in real time.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=SgUq-DN0By0
" target="_blank"><img src="http://img.youtube.com/vi/SgUq-DN0By0/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="560" height="315" border="0" /></a>  
(Click the image to redirect to object-detection-sptam video)

## Related Publications:
[1]  Taihú Pire, Javier Corti and Guillermo Grinblat.
**Online Object Detection and Localization on Stereo Visual SLAM System**
Journal of Intelligent & Robotic Systems, 2019.

## Table of Contents
- [License](#license)
- [Disclaimer](#disclaimer)
- [Dependencies](#dependencies)
- [Compilation](#compilation)
- [Run](#run)

# License

object-detection-sptam is released under GPLv3 license.

For a closed-source version of object-detection-sptam for commercial purposes, please contact the authors.

If you use *object-detection-sptam* in an academic work, please cite:

@article{pire2019object,  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  title = {{Online Object Detection and Localization on Stereo Visual SLAM System}},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  author = {Pire, Taih{\'u} and Corti, Javier and Grinblat, Guillermo},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  journal = {Journal of Intelligent {\&} Robotic Systems},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  day = {27},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  month = {August},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  year = {2019},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  issn = {1573-0409},  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  doi = {10.1007/s10846-019-01074-2}  
}

# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

# Dependencies

Move the content of ros directory (ros nodes and the network model that is used in rcnn_pose.py) to your ros workspace. Then compile the ros workspace.

## Install py-faster-rcnn and caffe-fast-rcnn

Clone the Faster R-CNN repository
     
     git submodule update --init --recursive 

and follow the installation instructions


## Modified S-PTAM Dependencies

### SuiteSparse
    
    sudo apt-get install libsuitesparse-dev

### g2o 
    
    cd  ~/object-detection-sptam/dependencies/
    source getG2o.sh
    cd ~/object-detection-sptam/dependencies/g2o
    mkdir build && cd build
    cmake ..
    make 
    sudo make install
    
### meta
    
    sudo cp -Rf ~/object-detection-sptam/dependencies/meta/include/meta /usr/include/

### pugixml
    
    cd ~/object-detection-sptam/dependencies/pugixml
    mkdir build
    cd build
    cmake ..
    make
    sudo make install/local
    
### ApproxMVBB
 
    cd ~/object-detection-sptam/dependencies/
    source getApproxMVBB.sh 
    cd  ~/object-detection-sptam/dependencies/ApproxMVBB
    mkdir build
    cd build 
    cmake ..
    make all
    sudo make install

# Compilation

    catkin build sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON
    
# Run

    roslaunch sptam dl_zed.launch

On execution is going to ask:
    
    ~/.local/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt

that is in the models_tained directory:

    models_tained/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt
    
copy or moved to python directory: 

    cp -Rf /data/object-detection-sptam/models_trained/* ~/.local/lib/python2.7/models/

