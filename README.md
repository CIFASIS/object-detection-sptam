*En progreso ...* 

En el directorio ros están los nodos de Ros y el modelo que usa para correr definido en rcnn_pose.py 
Mover el contenido del directorio ros al workspace de ros y compilar 

para correr:

    roslaunch sptam dl_zed.launch

En tiempo de ejecución pide por
    ~/.local/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt

que está en el directio models_tained:

    models_tained/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt
    
copiar o moverlos para que funcione.    

    cp -Rf /data/ondeloc/models_trained/* ~/.local/lib/python2.7/models/


    
    
*COMPILACIÓN*

**caffe-fast-rcnn**

*dependencia general Ubuntu 16.01*

    sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
    sudo apt-get install --no-install-recommends libboost-all-dev
    sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev
    
**Blas**
 
 **ATLAS**:
 
    apt-get install libatlas-base-dev
 
 **caffe**
 
    cp Makefile.config.example Makefile.config (revisar el Makefile.config y setear las variables necesarias) 
    mkdir build
    cd build
    cmake ..
    make -j4 && make pycaffe
    make install
    
 **py-faster-rcnn**
 
    cd lib
    make
 
 


Agregar 

    pip install --user easydict
    pip install --user skimage
    ln -s ~/ondeloc/py-faster-rcnn/caffe-fast-rcnn/build/install/python/caffe caffe
    ln -s ~/ondeloc/py-faster-rcnn/lib/fast_rcnn fast_rcnn
    ln -s ~/ondeloc/py-faster-rcnn/lib/utils utils
    ln -s ~/ondeloc/py-faster-rcnn/lib/nms nms
    ln -s ~/ondeloc/py-faster-rcnn/lib/rpn rpn


    
 

**dependencies**
Dependencias de sptam

**SuiteSparse**
    
    sudo apt-get install libsuitesparse-dev

  **g2o** 
    
    cd  ~/ondeloc/dependencies/
    source getG2o.sh
    cd ~/ondeloc/dependencies/g2o
    mkdir build && cd build
    cmake ..
    make 
    sudo make install
    
**meta**
    
    sudo cp -Rf ~/ondeloc/dependencies/meta/include/meta /usr/include/

**pugixml** 
    
    cd ~/ondeloc/dependencies/pugixml
    mkdir build
    cd build
    cmake ..
    make
    sudo make install/local
    
**ApproxMVBB**
 
    cd ~/ondeloc/dependencies/
    source getApproxMVBB.sh 
    cd  ~/ondeloc/dependencies/ApproxMVBB
    mkdir build
    cd build 
    cmake ..
    make all
    sudo make install
  




    
 
 
 
 
 
 
 
 