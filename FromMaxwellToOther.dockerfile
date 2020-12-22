FROM eevidal/object-detection-sptam-kinetic:ros-base-xenial-sptam-kinetic-maxwell

LABEL maintainer="Erica Vidal ericavidal@gmail.com"

RUN cd $HOME/object-detection-sptam && git pull \
    && cd $HOME/object-detection-sptam/py-faster-rcnn \
    && rm -Rf lib \
    && git checkout lib 

# the rigth arch have to be set on setup.py file (line 135)     
COPY ./py-faster-rcnn/lib/setup.py  /root/object-detection-sptam/py-faster-rcnn/lib/ 

RUN cd $HOME/object-detection-sptam/py-faster-rcnn/lib && make

CMD ["bash"]

WORKDIR catkin_ws