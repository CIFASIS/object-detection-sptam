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

# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

# Dependencies

Move the content of ros directory (ros nodes and the network model that is used in rcnn_pose.py) to your ros workspace. Then compile the ros workspace.

## py-faster-rcnn Dependencies

### caffe-fast-rcnn

*Tested on Ubuntu 16.01*

    sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
    sudo apt-get install --no-install-recommends libboost-all-dev
    sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
    
### Blas

    sudo apt-get install libblas-dev liblapack-dev
 
### ATLAS
 
    sudo apt-get install libatlas-base-dev
 
### Compile and Install Caffe
 
    cp Makefile.config.example Makefile.config (revisar el Makefile.config y setear las variables necesarias) 
    mkdir build
    cd build
    cmake ..
    make -j4 && make pycaffe
    make install
    
### Compile py-faster-rcnn
 
    cd ~/object-detection-sptam/py-faster-rcnn/lib
    make

### Python dependencies and softlinks creation

    pip install --user easydict
    pip install --user skimage
    cd ~/.local/lib/python2.7/site-packages
    ln -s ~/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn/build/install/python/caffe caffe
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/fast_rcnn fast_rcnn
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/utils utils
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/nms nms
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/rpn rpn
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/datasets datasets
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/roi_data_layer roi_data_layer
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/pycocotools pycocotools
    ln -s ~/object-detection-sptam/py-faster-rcnn/lib/transform transform

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

