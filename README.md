*En progreso ...* 

En el directorio ros están los nodos de Ros y el modelo que usa para correr definido en rcnn_pose.py 
Mover el contenido del directorio ros al workspace de ros y compilar 

para correr:

    roslaunch sptam dl_zed.launch

En tiempo de ejecución pide por
    ~/.local/lib/python2.7/models/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt

que está en el directio models_tained:

    models_tained/modelpose/VGG16/faster_rcnn_end2end/test.final.prototxt
    
    
*COMPILACIÓN*

**caffe-fast-rcnn**

    cp Makefile.config.example Makefile.config (revisar el Makefile.config) 
    mkdir build
    cd build
    cmake ..
    make -j4 && make pycaffe
    make install

    cp install/lib/* /usr/local/lib 
    cp -Rf install/include/ /usr/local/include
 
 
 **py-faster-rcnn**
 
    cd lib
    make
 
 **dependencies **
 
 
 
 
 
 
 
 