FROM ros:kinetic-ros-base-xenial

MAINTAINER Erica Vidal "ericavidal@gmail.com"

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*


# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-ros-base=1.3.2-0* 
   

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y 
RUN apt-get install  -y --no-install-recommends  \
    apt-utils \
    libprotobuf-dev \
    libleveldb-dev \
    libsnappy-dev \
    libopencv-dev \
    libhdf5-serial-dev \
    protobuf-compiler \
    libboost-all-dev  \
    libatlas-base-dev  \
    libgflags-dev \
    libgoogle-glog-dev \
    liblmdb-dev \
    ros-kinetic-pcl-ros \
    wget \
    vim


#Cuda 9
  
RUN wget http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_9.0.176-1_amd64.deb
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/libcudnn7_7.0.5.15-1+cuda9.0_amd64.deb
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/libcudnn7-dev_7.0.5.15-1+cuda9.0_amd64.deb
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/libnccl2_2.1.4-1+cuda9.0_amd64.deb
RUN wget http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/libnccl-dev_2.1.4-1+cuda9.0_amd64.deb
RUN dpkg -i cuda-repo-ubuntu1604_9.0.176-1_amd64.deb
RUN dpkg -i libcudnn7_7.0.5.15-1+cuda9.0_amd64.deb
RUN dpkg -i libcudnn7-dev_7.0.5.15-1+cuda9.0_amd64.deb
RUN dpkg -i libnccl2_2.1.4-1+cuda9.0_amd64.deb
RUN dpkg -i libnccl-dev_2.1.4-1+cuda9.0_amd64.deb
RUN apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
RUN apt-get update && apt-get install  --no-install-recommends -y  cuda=9.0.176-1 \
    && apt-get install libcudnn7-dev \
    && apt-get install libnccl-dev \
    && rm -rf /var/lib/apt/lists/*

# clone the code
RUN cd $HOME && git clone https://github.com/CIFASIS/object-detection-sptam.git \
    && cd $HOME/object-detection-sptam \
    && git checkout clean-the-code-kinetic \
    && git pull \
    && git submodule update --init --recursive 

#caffe & py-faster-rcnn
RUN apt-get update && apt-get install  -y --no-install-recommends python-pip \
    python-opencv  \
    ros-kinetic-opencv3 \
    && rm -rf /var/lib/apt/lists/* \
    && pip install wheel   \ 
    && pip install easydict==1.9 \
    && pip install setuptools \
    && pip install Pillow==5.0.0 \
    && pip install scipy==0.17.1 \
    && pip install PyWavelets==0.5.0 \
    && pip install networkx==1.9 \
    && pip install six==1.2.0 \
    && pip install matplotlib==1.5.0 \
    && pip install numpy==1.14.0 \
    && pip install scikit-image==0.9.3 \
    && pip install Cython==0.19.2 \
    && pip install ipython==3.1.0 \
    && pip install nose==1.3.7 \
    && pip install pandas==0.13.0 \
    && pip install python-dateutil==1.5 \
    && pip install PyYAML==3.11 \
    && pip install dask==0.12.0 \
    && pip install google==1.9.3 \
    && pip install protobuf==2.6.0
 
RUN cd $HOME/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn \
    && mkdir build && cd build &&  cmake -DUSE_CUDNN=1 -DUSE_NCCL=1 .. &&  make && make pycaffe && make install
RUN cd $HOME/object-detection-sptam/py-faster-rcnn/lib && make



   #     cmake \
   #     python-dev \
   #     python-numpy \
   #     python-setuptools \
   #     python-scipy 

#Install py-faster-rcnn and caffe
RUN cd /usr/lib/python2.7/dist-packages \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn/build/install/python/caffe caffe \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/fast_rcnn fast_rcnn \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/utils utils \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/nms nms \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/rpn rpn \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/datasets datasets \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/roi_data_layer roi_data_layer \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/pycocotools pycocotools \
    && ln -s $HOME/object-detection-sptam/py-faster-rcnn/lib/transform transform 


#sptam dependencies
RUN apt-get update && apt-get install libsuitesparse-dev -y

#build and install g2o
RUN cd $HOME/object-detection-sptam/g2o \
    && mkdir -p build && cd build \
    && cmake ..  \
    && make && make install 

#install meta
RUN cp -Rf $HOME/object-detection-sptam/dependencies/meta/include/meta /usr/include/

#Build pugixml
RUN cd $HOME/object-detection-sptam/dependencies/pugixml \
    &&  mkdir build && cd build \
    &&  cmake .. \
    &&  make && make install/local

#Build and install ApproxMVBB
RUN cd $HOME/object-detection-sptam/ApproxMVBB \
    &&  mkdir build && cd build \
    &&  cmake .. \
    &&  make all && make install \
    && cp -Rf $HOME/object-detection-sptam/ApproxMVBB/build/install/include /usr/include/ \
    && cp -Rf $HOME/object-detection-sptam/ApproxMVBB/build/install/lib/* /usr/lib/


RUN apt-get install python-catkin-tools -y \
    ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-cv-bridge \
    ros-kinetic-image-geometry \    
    ros-kinetic-image-transport \
    qt5-default \
    libqt5opengl5-dev \
    libqglviewer-dev 
   

#Create actkin worksapce
RUN . /opt/ros/kinetic/setup.sh \
    && mkdir -p catkin_ws/src \
    && cd catkin_ws \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=kinetic -y 
    

#Build workspace and sptam
RUN . /opt/ros/kinetic/setup.sh \
    && cd catkin_ws/src \
    && catkin init \
    && ln -s $HOME/object-detection-sptam/ros/ros-utils  ros_utils \
    && ln -s $HOME/object-detection-sptam/ros/sptam sptam \
    && ln -s $HOME/object-detection-sptam/ros/dl_node dl_node \
    && cd .. \
    && catkin build ros_utils \
    && catkin build sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON \
    && . devel/setup.sh

COPY ./data/caffeModels/pose_coco_Allconst_iter16000.caffemodel  $HOME/object-detection-sptam/data/caffeModels/
RUN  rm -rf /var/lib/apt/lists/* \
CMD ["bash"]


WORKDIR catkin_ws
