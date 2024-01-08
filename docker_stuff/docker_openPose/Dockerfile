FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04
LABEL maintainer="Ettore Landini"

ARG ros_distro

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_UPDATED_ON=28_09_2022

# Install essentials
RUN apt-get update && apt-get install -y apt-utils software-properties-common sudo psmisc lsb-release protobuf-compiler libatlas-base-dev \
    tmux nano geany vim wget curl build-essential git gitk cmake cmake-curses-gui autoconf xserver-xorg-video-dummy xserver-xorg-legacy \
    net-tools terminator libjpeg-dev ffmpeg apt-transport-https ca-certificates gnupg libace-dev ycm-cmake-modules locales \
    python3-setuptools python3-pip iproute2 python3-tornado swig lsof iftop iputils-ping gdb bash-completion btop mlocate && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy bpytop scikit-learn==0.24.2 opencv-python

RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./common/xorg.conf /etc/X11/xorg.conf

# Locale config
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install yarp dependencies
RUN apt-get update && apt-get install -y libode8 libace-dev libboost-filesystem-dev libboost-system-dev \
    libboost-thread-dev libc6 python3 libgsl0-dev libncurses5-dev libsdl1.2-dev subversion git gfortran \
    libxmu-dev libode-dev wget unzip qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5svg5 \
    libqt5opengl5-dev libopencv-dev freeglut3-dev libtinyxml-dev libblas-dev coinor-libipopt-dev \
    liblapack-dev libmumps-dev qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls \
    qml-module-qt-labs-settings libqcustomplot-dev qml-module-qt-labs-folderlistmodel libedit-dev \
    libeigen3-dev libjpeg-dev libsimbody-dev libxml2-dev libjs-underscore portaudio19-dev libgtkmm-2.4-dev \
    libi2c-dev libv4l-dev

# Create user: user1
USER root
RUN useradd -l -u 33334 -G sudo -md /home/user1 -s /bin/bash -p user1 user1 && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Set ENV
USER user1
RUN mkdir /home/user1/robotology
ENV robotology_install_user user1
ENV user1_home /home/$robotology_install_user
ENV robotology_install_folder $user1_home/robotology

RUN sudo apt-get update && sudo apt-get install -y libgoogle-glog-dev libhdf5-dev

# Build openpose
USER $robotology_install_user
WORKDIR $user1_home
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose && \
    cd ./openpose/ && \
    git submodule update --init --recursive --remote && \
    git checkout tags/v1.7.0
RUN cd $user1_home/openpose && \
    mkdir build && \
    cd ./build && \
    # Right now the following does not work.
    # https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/2233
    # You have to download the weights manually
    # cmake .. -DDOWNLOAD_HAND_MODEL=OFF -DUSE_CUDNN=OFF && \
    cmake .. -DDOWNLOAD_HAND_MODEL=OFF -DDOWNLOAD_BODY_25_MODEL=OFF -DDOWNLOAD_FACE_MODEL=OFF -DUSE_CUDNN=OFF && \
    make -j11
ENV openpose_ROOT=$user1_home/openpose


# Build ycm
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/ycm.git -b master
RUN cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release && \
    make -j11
ENV YCM_DIR=/home/user1/robotology/ycm/build

# Build YARP
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/yarp.git -b yarp-3.9
RUN cd yarp && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DYARP_COMPILE_libYARP_math=ON \
    -DYARP_COMPILE_GUIS=ON \
    -DYARP_COMPILE_DEVICE_PLUGINS=ON \
    -DENABLE_yarpcar_mjpeg=ON \
    -DENABLE_yarpcar_human=ON \
    -DENABLE_yarpcar_segmentationimage=ON \
    -DENABLE_yarpcar_portmonitor=ON \
    -DENABLE_yarpmod_controlboardwrapper2=ON \
    -DENABLE_yarpmod_laserFromDepth=ON \
    -DENABLE_yarpmod_laserFromExternalPort=ON \
    -DENABLE_yarpmod_laserFromDepth=ON \
    -DENABLE_yarpmod_laserFromPointCloud=ON \
    -DENABLE_yarpmod_laserFromRosTopic=ON \
    -DENABLE_yarppm_soundfilter_resample=ON \
    -DENABLE_yarppm_bottle_compression_zlib=ON \
    -DENABLE_yarppm_depthimage_compression_zlib=ON \
    -DENABLE_yarppm_image_compression_ffmpeg=ON \
    -DENABLE_yarppm_depthimage_to_mono=ON \
    -DENABLE_yarppm_depthimage_to_rgb=ON && \
    make -j11
ENV YARP_ROOT=$robotology_install_folder/yarp
ENV YARP_DIR=$robotology_install_folder/yarp/build

# YARP bindings
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN cd yarp && cd bindings &&  mkdir build && cd build && cmake .. -DCREATE_PYTHON=ON && make -j11

# Install YARP completion
RUN sudo ln -s /usr/local/share/bash-completion/completions/yarp /usr/share/bash-completion/completions

# Build icub-main
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-main.git -b devel
RUN cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DICUBMAIN_COMPILE_SIMULATORS=OFF \
    -DENABLE_icubmod_gazecontrollerclient=ON && \
    make -j11
ENV ICUB_ROOT=$robotology_install_folder/icub-main
ENV ICUB_DIR=$robotology_install_folder/icub-main/build

# Build icub-contrib-common
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-contrib-common.git
RUN cd icub-contrib-common && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$robotology_install_folder/iCubContrib && sudo make -j11 install
ENV ICUBcontrib_DIR=$robotology_install_folder/iCubContrib/lib
ENV ICUBcontrib_SHARE=$robotology_install_folder/iCubContrib/share/ICUBcontrib

# Build yarpopenpose (Maria Lombardi)
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/human-sensing
RUN cd human-sensing/yarpOpenPose && \
    mkdir build && \
    cd ./build && \
    cmake .. && \
    make && sudo make install

# Build mutual gaze classifier (Maria Lombardi)
USER $robotology_install_user
WORKDIR $user1_home
RUN git clone https://github.com/MariaLombardi/mutual-gaze-classifier-demo.git
RUN cd mutual-gaze-classifier-demo && \
    mkdir build && \
    cd ./build && \
    cmake .. && \
    make && sudo make install

# Set environmental variables
USER $robotology_install_user
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/$robotology_install_user/.bashrc
ENV PATH=$PATH:$YARP_DIR/bin:$navigation_DIR/bin:$CER_DIR/bin:$ICUB_DIR/bin:$ICUB_DIR:$robotology_install_folder/iCubContrib/bin:/home/user1/robotology/Groot/build:/home/user1/tour-guide-robot/build/bin
ENV DISPLAY=:1
ENV YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUB_DIR/share/iCub:$navigation_DIR/share/navigation:$user1_home/tour-guide-robot/build/share/tour-guide-robot:$CER_DIR/share/CER:$ICUBcontrib_DIR/share/ICUBcontrib:$robotology_install_folder/yarp-devices-ros2/build/share/yarp:$robotology_install_folder/yarp-device-rplidar/build/share/yarp:$ICUBcontrib_SHARE
ENV LD_LIBRARY_PATH=$robotology_install_folder/yarp/build/lib/yarp/
ENV YARP_COLORED_OUTPUT=1
ENV QT_X11_NO_MITSHM=1
ENV PYTHONPATH=$PYTHONPATH:/home/user1/robotology/yarp/bindings/build/lib/python3/
ENV CYCLONEDDS_URI=/home/user1/tour-guide-robot/app/navigation2/conf/cyclone_dds_settings.xml

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

# Navigation stuff
WORKDIR ${user1_home}
RUN git clone https://github.com/elandini84/yarp-web-teleop.git -b master
RUN git clone https://github.com/hsp-iit/tour-guide-robot -b $ros_distro
RUN /bin/bash -c "cd tour-guide-robot; mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release -DBTState=OFF"

# R1 NETWORK CONFIG
RUN yarp conf 192.168.100.10 10000

WORKDIR ${user1_home}/tour-guide-robot
RUN sudo updatedb
# Change the lidar devices permissions inside the docker to be able to start the devices with yarprobotinterface
CMD if [ -e /dev/ttyUSB0 ]; then sudo chown user1:user1 /dev/ttyUSB0 && sudo chown user1:user1 /dev/ttyUSB1; fi && bash
