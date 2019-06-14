branch

DIRECTORIES:
utils: has different utils, as logging, Profilling, and math helpers.
StereoProcessing: tiene funciones para el procesamiento de imagenes estereo
Gui: tiene todas las funciones correspondientes a la interfaz de usuario para visualizar imagenes y features (Visuzalizacion).
Configuration: tiene funciones para leer el archivo de configuracion YML de entrada
FrameGenerator: Tiene las funciones para leer los frames de un directorio de imagenes, video o de una camara
Matching: Tiene todas las funciones que forman parte del matching
Localization: tiene todas las clases que forman parte de la localizacion y el mapeo
Test: contiene todos los test del codigo que fui haciendo durante el desarrollo

DEPENDENCES:
ubuntu 14.04
OpenCV (from PPA ppa:xqms/opencv-nonfree)
ROS indigo (instala PCL)

COMPILATION:
$ mkdir build
$ cd build
$ cmake ..
$ make

DATASETS NOTES:
WebPage: http://www.cvlibs.net/datasets/karlsruhe_sequences/
Download:
http://www.mrt.uni-karlsruhe.de/geigerweb/cvlibs.net/karlsruhe_sequences/2010_03_09_drive_0019.zip
Download:
http://www.mrt.uni-karlsruhe.de/geigerweb/cvlibs.net/karlsruhe_sequences/2009_09_08_drive_0019.zip

WebPage: http://www.rawseeds.org/home/
Download (by Torrent):
http://www.rawseeds.org/rs/capture_sessions/view/7

webpage:http://www.mrpt.org/robotics_datasets

Para generar el archivo que lista los nombres de imagenes a ser leidas utilizar el comando por consola:

find `pwd` -name "*.png" | sort >> file.txt

dentro del directorio de dichas imagenes.

Para borrar achivos SVN de un directoro y de sus subdirectorios hacer:

find . -iname .svn -exec rm { } \;

PCL VISUALIZATION NOTES:

Para poder seleccionar un punto en el visualizador de PCL:
Shift + Left Click

Para poner la camara en una posicion deseada. Durante la visualizacion presionar la tecla "c"
retorno por consola:  "5.2446,5244.6/47.1943,13.7354,313.345/-701.487,-89.8187,369.288/0.137336,-0.990515,0.00444695/0.8575/683,384/1,52"
como usar PCL setcamera
 clip[0],clip[1]/focal[0],focal[1]/focal[2]/pos[0],pos[1],pos[2]/view[0],view[1],view[2]/FOV/window_size[0],window_size[1]/ window_pos[0],window_pos[1]
 viewer->setCameraPosition(focal[0],focal[1],focal[2],pos[0],pos[1],pos[2],view[0],view[1],view[2])
 viewer->setCameraPosition(47.1943,13.7354,313.345,-701.487,-89.8187,369.288,0.137336,-0.990515,0.00444695);
 viewer->setCameraFieldOfView(0.8575);
 viewer->setCameraClipDistances(5.2446,5244.6);
 viewer->setPosition(1,52);
 viewer->setSize(683,384);


GEOMETRY NOTES:
Matrices:

Transformacion del mundo a la Camara:

T_cw =  R t
	0 1

Transformacion de la camara al mundo:

T_wc = R^T -R^Tt
        0    1

Posicion de la camara en el mundo

C = -R^Tt

Orientacion de la camara en el mundo:

orientacion = R^T

PAGINAS con INFORMACION sobre TUKEY:

http://research.microsoft.com/en-us/um/people/zhang/INRIA/Publis/Tutorial-Estim/node24.html
http://webmining.spd.louisville.edu/Websites/tutorials/RobustStatistics/RobustStatistics.html
http://research.microsoft.com/en-us/um/people/zhang/INRIA/Publis/Tutorial-Estim/node25.html
la tesis te klein tambien lo cuenta bien

NOTES ABOUT ROS

roslaunch stam/launch/stam.launch

con QtCreator hay que pasar el cmake con esta bandera
-DCATKIN_DEVEL_PREFIX=/path/to/catkin_workspace/devel

para correr con el servidor i7 hacer:
ssh barracuda.cuartos.inv.dc.uba.ar
usuario: taihu
pass: eitileda

KITTI BENCHMARK:

ejecutar ./evaluate_odometry prueba

donde el directorio prueba se encuentra en la ruta: /devkit/cpp/results/prueba
el directorio prueba tiene los siguintes directorios con los siguientes archivos:

+--devkit/
   +--cpp/
      +--data/
      |  +--odometry/
      |     +--poses/
      |        +--00.txt
      |        +--...
      |        +--nn.txt // archivos de ground-truth dados por el dataset
      +--results/
         +--prueba/
            +--data/
            +--00.txt
            +--...
            +--nn.txt // resultados dados por el metodo de SLAM

Como obtener las poses del archivo log
grep KITTI_POSE: 00.log > 00.txt
luego nos quedamos con las columnas que nos sirven (orientacion y posicion)
cut -d' ' -f3- 00.txt >> 00.txt
Una vez que tengamos las poses tenemos que situarlas en /devkit/cpp/results/prueba/data/
Tambien tenemos que poner las poses del ground-truth dadas por el dataset en el directorio /devkit/cpp/data/odometry/poses/






