*README in progress.* 

*object-detection-sptam* is a SLAM system for stereo cameras which builds a map of objects in a scene. The system is based on the SLAM method S-PTAM and an object detection module. The object detection module uses Deep Learning to perform online detection and provide the 3d pose estimations of objects present in an input image, while S-PTAM estimates the camera pose in real time.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=SgUq-DN0By0
" target="_blank"><img src="http://img.youtube.com/vi/SgUq-DN0By0/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="560" height="315" border="0" /></a>  
(Click the image to redirect to S-PTAM video)

## Related Publications:
[1]  Taihú Pire, Javier Corti and Guillermo Grinblat.
**Online Object Detection and Localization on Stereo Visual SLAM System**
Journal of Intelligent & Robotic Systems, 2019.

## Table of Contents
- [License](#license)
- [Disclaimer](#disclaimer)
- [Dependencies](#dependencies)

# License

object-detection-sptam is released under GPLv3 license.

For a closed-source version of object-detection-sptam for commercial purposes, please contact the authors.

# Disclaimer
This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

Move the content of ros directory (ros nodes and the network model that is used in rcnn_pose.py) to your ros workspace. Then compile the ros workspace.

En el directorio ros están los nodos de Ros y el modelo que usa para correr definido en rcnn_pose.py 
Mover el contenido del directorio ros al workspace de ros y compilar 

# Run

    roslaunch sptam dl_zed.launch

En tiempo de ejecución pide por
    ~/.local/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt

que está en el directio models_tained:

    models_tained/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt
    
copiar o moverlos para que funcione.    

    cp -Rf /data/ondeloc/models_trained/* ~/.local/lib/python2.7/models/


# Compilation

## caffe-fast-rcnn

*dependencia general Ubuntu 16.01*

    sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
    sudo apt-get install --no-install-recommends libboost-all-dev
    sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
    
## Blas
 
## ATLAS
 
    apt-get install libatlas-base-dev
 
## caffe
 
    cp Makefile.config.example Makefile.config (revisar el Makefile.config y setear las variables necesarias) 
    mkdir build
    cd build
    cmake ..
    make -j4 && make pycaffe
    make install
    
## py-faster-rcnn
 
    cd lib
    make
 

## Python dependencies and softlinks creation

    pip install --user easydict
    pip install --user skimage
    cd ~/.local/lib/python2.7/site-packages
    ln -s ~/ondeloc/py-faster-rcnn/caffe-fast-rcnn/build/install/python/caffe caffe
    ln -s ~/ondeloc/py-faster-rcnn/lib/fast_rcnn fast_rcnn
    ln -s ~/ondeloc/py-faster-rcnn/lib/utils utils
    ln -s ~/ondeloc/py-faster-rcnn/lib/nms nms
    ln -s ~/ondeloc/py-faster-rcnn/lib/rpn rpn
    ln -s ~/ondeloc/py-faster-rcnn/lib/datasets datasets
    ln -s ~/ondeloc/py-faster-rcnn/lib/roi_data_layer roi_data_layer
    ln -s ~/ondeloc/py-faster-rcnn/lib/pycocotools pycocotools
    ln -s ~/ondeloc/py-faster-rcnn/lib/transform transform

# S-PTAM Dependencies

## SuiteSparse
    
    sudo apt-get install libsuitesparse-dev

## g2o 
    
    cd  ~/ondeloc/dependencies/
    source getG2o.sh
    cd ~/ondeloc/dependencies/g2o
    mkdir build && cd build
    cmake ..
    make 
    sudo make install
    
## meta
    
    sudo cp -Rf ~/ondeloc/dependencies/meta/include/meta /usr/include/

## pugixml
    
    cd ~/ondeloc/dependencies/pugixml
    mkdir build
    cd build
    cmake ..
    make
    sudo make install/local
    
## ApproxMVBB
 
    cd ~/ondeloc/dependencies/
    source getApproxMVBB.sh 
    cd  ~/ondeloc/dependencies/ApproxMVBB
    mkdir build
    cd build 
    cmake ..
    make all
    sudo make install
  
##########################

  catkin build sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON
