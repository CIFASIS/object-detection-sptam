*object-detection-sptam* is a SLAM system for stereo cameras which builds a map of objects in a scene. The system is based on the SLAM method S-PTAM and an object detection module. The object detection module uses Deep Learning to perform online detection and provide the 3d pose estimations of objects present in an input image, while S-PTAM estimates the camera pose in real time.

## Video
<a href="https://youtu.be/SgUq-DN0By0" target="_blank">
  <img src="https://github-production-user-asset-6210df.s3.amazonaws.com/6648400/260782848-7846840d-85b0-4b2c-94a5-da044c158ce5.png" alt="object-detection-sptam" width="700" />
</a>

## Related Publications:
[1]  Taihú Pire, Javier Corti and Guillermo Grinblat.
**Online Object Detection and Localization on Stereo Visual SLAM System**
Journal of Intelligent & Robotic Systems, 2019.

## Table of Contents
- [License](#license)
- [Disclaimer](#disclaimer)
- [System requirements](#requirements)
- [Dependencies](#dependencies)
- [Compilation](#compilation)
- [Run](#run)
- [Docker](#docker)

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


# Requirements
 ### Hardware
    * Nvidia GPU Graphic card 
 ### System 
    * Ubuntu 16.04
    * ros-kinectic
    * Nvidia drivers
    * cuda-8, cuda-9 or cuda-10
    * cudnn5,cudnn6 or cudnn7


# Dependencies

Move the content of ros directory (ros nodes and the network model that is used in rcnn_pose.py) to your ros workspace. Then compile the ros workspace.

## py-faster-rcnn Dependencies
### caffe-fast-rcnn

    sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
    sudo apt-get install --no-install-recommends libboost-all-dev
    sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev

### Blas

    sudo apt-get install libblas-dev liblapack-dev

### ATLAS

    sudo apt-get install libatlas-base-dev
    
### Python packages

    pip install wheel  
    pip install easydict==1.9 
    pip install setuptools 
    pip install Pillow==5.0.0 
    pip install scipy==0.17.1 
    pip install PyWavelets==0.5.0 
    pip install networkx==1.9 
    pip install six==1.2.0 
    pip install matplotlib==1.5.0 
    pip install numpy==1.14.0 
    pip install Cython==0.19.2 
    pip install scikit-image==0.9.3 
    pip install ipython==3.1.0 
    pip install nose==1.3.7 
    pip install pandas==0.13.0 
    pip install python-dateutil==1.5 
    pip install PyYAML==3.11 
    pip install dask==0.12.0 
    pip install google==1.9.3 
    pip install protobuf==2.6.0

### Compile and Install Caffe
    cd ~/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn
    cp Makefile.config.example Makefile.config (edit the Makefile.config file and set the vars) 
    mkdir build
    cd build
    cmake ..
    make -j4 && make pycaffe
    make install

### Compile py-faster-rcnn

    cd ~/object-detection-sptam/py-faster-rcnn/lib

Edit setup.py and set appropiate sm arch code for your GPU ([see here]( https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/ ).)

     make


## Modified S-PTAM Dependencies

    git submodule update --init --recursive 

### Install SuiteSparse, opencv3, intel tbb library, ros pcl package 
    
    sudo apt-get install libsuitesparse-dev  python-opencv  ros-kinetic-opencv3  libtbb-dev  ros-kinetic-pcl-ros


### g2o 
    
    cd ~/object-detection-sptam/g2o
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
 
    cd  ~/object-detection-sptam/ApproxMVBB
    mkdir build
    cd build 
    cmake ..
    make all
    sudo make install

# Sptam Compilation

    catkin build sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON
    
# Run

    roslaunch sptam dl_zed.launch

On execution is going to ask:
    
    ~/.local/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt

that is in the models_tained directory:

    models_tained/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt
    
copy or moved to python directory: 

    cp -Rf /data/object-detection-sptam/models_trained/* ~/.local/lib/python2.7/models/

# Docker

## Build image from Dockerfile

#### 0) Install [nvidia-container-runtime](https://github.com/NVIDIA/nvidia-container-runtime)
#### 1) clone object-detection-sptam 
    
    git clone https://github.com/CIFASIS/object-detection-sptam.git 
    git checkout clean-the-code-kinetic
    git pull

#### 2) Download the caffemodel file
    cd object-detection-sptam
    source data/caffeModels/getCaffeModel.sh

#### 3) Edit  Makefile.config of caffe and setup.py of 
    cd ~/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn
    cp Makefile.config.example Makefile.config 
Edit the Makefile.config file and set the vars. 
    
Edit setup.py and set appropiate sm arch code for your GPU ([see]( https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/ ).)
    
    cd ~/object-detection-sptam/py-faster-rcnn/lib


#### 4) Build docker image: 
    
    sudo docker build -t "object-detection-sptam:kinetic" .   


### Run

The resulting image can be seen with the docker images command.

    docker images 

Once the image is built, we can launch the container with the docker run command.

    docker run -it --name sptam_container --rm --gpus all object-detection-sptam:kinetic bash

This starts an interactive bash shell in the container once it is initialized.

We can launch other terminal to conect to the container with the next command:

    docker container exec -it sptam_container bash

For play rosbags from the host into the container we can mount the folder that contains the rosbags file with --volume argument:

    sudo docker run --volume=<PATH-TO-ROSBAGS-IN-THE-HOST>:/rosbags -it  --rm --gpus all object-detection-sptam:kinetic bash

And we need the caffemodel file to, so we can mount the path where is the file pose_coco_Allconst_iter16000.caffemodel: 

    docker run --volume=<PATH-TO-ROSBAGS-IN-THE-HOST>:/rosbags --volume=<PATH-TO-CAFFEMODEL>:/root/object-detection-sptam/data/caffeModel -it  --rm --gpus all object-detection-sptam:kinetic bash
  
  
## Pull image from dockerhub    

For Quadro M6000 , GeForce 900, GTX-970, GTX-980 or GTX Titan X cards only. 

    docker pull eevidal/object-detection-sptam-kinetic:ros-base-xenial-sptam-kinetic-maxwell
    
For GTX 1080, GTX 1070, GTX 1060, GTX 1050, GTX 1030, Titan Xp, Tesla P40, Tesla P4, Discrete GPU on the NVIDIA Drive PX2

    docker pull eevidal/object-detection-sptam-kinetic:ros-base-xenial
      
For other GPUs use FromMaxwellToOther.dockerfile to create your own 
