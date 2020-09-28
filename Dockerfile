#FROM nvidia/cuda:10.2-base
FROM ros:kinetic-ros-base-xenial

LABEL maintainer="Erica Vidal ericavidal@gmail.com"

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    ros-kinetic-ros-base=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*
    
   

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
    && rm -rf /var/lib/apt/lists/*


#Install Cuda 10, cudnn7 and nccl 2

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates apt-transport-https gnupg-curl && \
    NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
    NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \
    apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \
    echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge --auto-remove -y gnupg-curl \
    && rm -rf /var/lib/apt/lists/*



ENV CUDA_VERSION 10.2.89
ENV CUDA_PKG_VERSION 10-2=$CUDA_VERSION-1
ENV NCCL_VERSION 2.7.8
ENV CUDNN_VERSION 7.6.5.32

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-2 \
    cuda-nvtx-$CUDA_PKG_VERSION \
    cuda-nvml-dev-$CUDA_PKG_VERSION \
    cuda-command-line-tools-$CUDA_PKG_VERSION \
    cuda-nvprof-$CUDA_PKG_VERSION \
    cuda-libraries-$CUDA_PKG_VERSION \
    cuda-npp-$CUDA_PKG_VERSION \
    cuda-npp-dev-$CUDA_PKG_VERSION \
    cuda-libraries-dev-$CUDA_PKG_VERSION \
    cuda-minimal-build-$CUDA_PKG_VERSION \
    libcublas10=10.2.2.89-1 \
    libcublas-dev=10.2.2.89-1 \
    libnccl2=$NCCL_VERSION-1+cuda10.2 \
    libnccl-dev=2.7.8-1+cuda10.2 \
    libcudnn7=$CUDNN_VERSION-1+cuda10.2 \
    libcudnn7-dev=$CUDNN_VERSION-1+cuda10.2 \
    && ln -s cuda-10.2 /usr/local/cuda \
    && apt-mark hold libcudnn7 \ 
    && apt-mark hold libnccl2 \
    && rm -rf /var/lib/apt/lists/*

 
ENV CUDA_HOME=/usr/local/cuda-10.2
ENV LD_LIBRARY_PATH=${CUDA_HOME}/lib64:/usr/local/cuda/lib64/stubs 
ENV PATH=${CUDA_HOME}/bin:${PATH}

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
    && pip install Cython==0.19.2 \
    && pip install scikit-image==0.9.3 \
    && pip install ipython==3.1.0 \
    && pip install nose==1.3.7 \
    && pip install pandas==0.13.0 \
    && pip install python-dateutil==1.5 \
    && pip install PyYAML==3.11 \
    && pip install dask==0.12.0 \
    && pip install google==1.9.3 \
    && pip install protobuf==2.6.0


# clone object-detection-sptam code
RUN cd $HOME && git clone https://github.com/CIFASIS/object-detection-sptam.git \
    && cd $HOME/object-detection-sptam \
    && git checkout clean-the-code-kinetic \
    && git pull 
RUN  cd $HOME/object-detection-sptam &&  git submodule update --init --recursive 



ENV NVIDIA_VISIBLE_DEVICES=0
RUN mkdir -p temp
COPY ./py-faster-rcnn/caffe-fast-rcnn/Makefile.config temp/
COPY ./py-faster-rcnn/lib/setup.py temp/
RUN cp temp/Makefile.config $HOME/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn/  \
    && cp temp/setup.py $HOME/object-detection-sptam/py-faster-rcnn/lib/ \
    && cd $HOME/object-detection-sptam/py-faster-rcnn/caffe-fast-rcnn \
    && mkdir build && cd build &&  cmake -DUSE_CUDNN=1 .. &&  make && make pycaffe && make install \
    && cd $HOME/object-detection-sptam/py-faster-rcnn/lib \
    && make \
    && rm -rf /temp

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
RUN apt-get update && apt-get install libsuitesparse-dev -y \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*
    


#build and install g2o
RUN cd $HOME/object-detection-sptam/g2o \
    && mkdir -p build && cd build \
    && cmake ..  \
    && make && make install \
    && rm -rf $HOME/object-detection-sptam/g2o/build/*

#install meta
RUN cp -Rf $HOME/object-detection-sptam/dependencies/meta/include/meta /usr/include/

#Build pugixml
RUN cd $HOME/object-detection-sptam/dependencies/pugixml \
    &&  mkdir build && cd build \
    &&  cmake .. \
    &&  make && make install/local \
    && rm -rf $HOME/object-detection-sptam/dependencies/pugixml/build/*

#Build and install ApproxMVBB
RUN cd $HOME/object-detection-sptam/ApproxMVBB \
    &&  mkdir build && cd build \
    &&  cmake .. \
    &&  make all && make install \
    && cp -Rf $HOME/object-detection-sptam/ApproxMVBB/build/install/include /usr/include/ \
    && cp -Rf $HOME/object-detection-sptam/ApproxMVBB/build/install/lib/* /usr/lib/ 
 

RUN apt-get update && apt-get install python-catkin-tools -y \
    ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-cv-bridge \
    ros-kinetic-image-geometry \    
    ros-kinetic-image-transport \
    qt5-default \
    libqt5opengl5-dev \
    libqglviewer-dev \
    ros-kinetic-stereo-image-proc \
    && rm -rf /var/lib/apt/lists/*
    
   

#Create catkin worksapce
RUN . /opt/ros/kinetic/setup.sh \
    && mkdir -p catkin_ws/src \
    && cd catkin_ws \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=kinetic -y  \
    && rm -rf /var/lib/apt/lists/* 
    

#Build workspace and sptam
RUN . /opt/ros/kinetic/setup.sh \
    && cd catkin_ws/src \
    && catkin init \
    && ln -s $HOME/object-detection-sptam/ros/ros-utils  ros_utils \
    && ln -s $HOME/object-detection-sptam/ros/sptam sptam \
    && ln -s $HOME/object-detection-sptam/ros/dl_node dl_node \
    && cd .. \
    && catkin build ros_utils \
    && catkin build sptam -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSINGLE_THREAD=OFF -DSHOW_TRACKED_FRAMES=ON -DSHOW_PROFILING=ON -DPARALLELIZE=ON 

#copy caffemodel file
#COPY ./data/caffeModels/pose_coco_Allconst_iter16000.caffemodel  temp 
#RUN cp temp/pose_coco_Allconst_iter16000.caffemodel $HOME/object-detection-sptam/data/caffeModels/

#copy models


RUN mkdir -p /usr/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end
RUN cp $HOME/object-detection-sptam/models_trained/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt /usr/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end
RUN mkdir -p /usr/lib/python2.7/models/pascal_voc
RUN cd /usr/lib/python2.7/models/pascal_voc \
    && ln -s /usr/lib/python2.7/models/modelpose modelpose


CMD ["bash"]

WORKDIR catkin_ws
