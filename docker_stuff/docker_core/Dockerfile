FROM ubuntu:20.04
LABEL maintainer="Konstantinos Karapas"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_UPDATED_ON=30_03_2022

# Install essentials
RUN apt-get update && apt-get install -y apt-utils software-properties-common sudo psmisc lsb-release \
    tmux nano geany vim tilde wget curl build-essential git gitk cmake cmake-curses-gui \
    libedit-dev libxml2-dev autoconf libtool pkg-config \
    net-tools terminator libopencv-dev lsof iftop iputils-ping gdb bash-completion \
    libjpeg-dev ffmpeg apt-transport-https ca-certificates gnupg libgoogle-glog-dev libboost-all-dev libhdf5-serial-dev libatlas-base-dev\
    epstool transfig tilde python3-setuptools python3-pip swig python3-tornado \
    libsoxr-dev sox xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy chrony iproute2 locate

RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./common/xorg.conf /etc/X11/xorg.conf

RUN pip3 install webrtcvad numpy bpytop scikit-learn

# Install yarp dependencies
RUN echo "deb http://www.icub.org/ubuntu `lsb_release -cs` contrib/science" > /etc/apt/sources.list.d/icub.list
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 57A5ACB6110576A6
RUN apt-get update && apt-get install -y icub-common 

# Install Qt patch for Xfce
RUN apt-get install -y qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings

# Select branch + build type
ARG BUILD_TYPE=Release

# ROS installation
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full  \
    ros-noetic-nav-core ros-noetic-map-server ros-noetic-gmapping ros-noetic-amcl \
    ros-noetic-move-base python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool ros-noetic-tf2-sensor-msgs
RUN rosdep init

# create another user called user1
USER root
RUN useradd -l -u 33334 -G sudo -md /home/user1 -s /bin/bash -p user1 user1 && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# ROS installation (part2)
USER user1
RUN sudo rosdep fix-permissions && rosdep update && echo "source /opt/ros/noetic/setup.bash" >> /home/user1/.bashrc && /bin/bash -c "source /home/user1/.bashrc"

# set install_folder
USER user1
RUN mkdir /home/user1/robotology
ENV robotology_install_user user1
ENV user_home /home/$robotology_install_user
ENV robotology_install_folder $user_home/robotology

# CUDA and cuDNN (has to be those versions)
USER $robotology_install_user
WORKDIR $user_home
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin && sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/3bf863cc.pub
RUN sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
RUN sudo apt-get update && sudo apt-get -y install cuda-toolkit-10-0

RUN wget https://developer.download.nvidia.com/compute/redist/cudnn/v7.6.1/Ubuntu18_04-x64/libcudnn7_7.6.1.34-1+cuda10.0_amd64.deb
RUN sudo dpkg -i libcudnn7_7.6.1.34-1+cuda10.0_amd64.deb
RUN wget https://developer.download.nvidia.com/compute/redist/cudnn/v7.6.1/Ubuntu18_04-x64/libcudnn7-dev_7.6.1.34-1+cuda10.0_amd64.deb
RUN sudo dpkg -i libcudnn7-dev_7.6.1.34-1+cuda10.0_amd64.deb
RUN sudo rm libcudnn7_7.6.1.34-1+cuda10.0_amd64.deb libcudnn7-dev_7.6.1.34-1+cuda10.0_amd64.deb

# Build ycm
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/ycm.git -b master
RUN cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    make -j11

# GCC Compiler (because openpose requires this version)
RUN sudo apt install -y gcc-7 g++-7
RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7 --slave /usr/bin/g++ g++ /usr/bin/g++-7

# Build openpose (Maria Lombardi)
USER $robotology_install_user
WORKDIR $user_home
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose && \
    cd ./openpose/ && \
    git submodule update --init --recursive --remote && \
    git checkout tags/v1.6.0
RUN cd $user_home/openpose && \
    mkdir build && \
    cd ./build && \
    cmake .. -DDOWNLOAD_HAND_MODEL=OFF && \
    make -j11
ENV openpose_ROOT=$user_home/openpose
RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9 --slave /usr/bin/g++ g++ /usr/bin/g++-9

# Build yarp
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/yarp.git -b master
RUN cd yarp && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DYARP_COMPILE_libYARP_math=ON \
    -DYARP_COMPILE_GUIS=ON \
    -DYARP_COMPILE_DEVICE_PLUGINS=ON \
    -DENABLE_yarpcar_mjpeg=ON \
    -DENABLE_yarpcar_depthimage=ON \
    -DENABLE_yarpcar_depthimage2=ON \
    -DENABLE_yarpcar_segmentationimage=ON \
    -DENABLE_yarpcar_portmonitor=ON \
    -DENABLE_yarpmod_portaudio=ON \
    -DENABLE_yarpmod_portaudioPlayer=ON \
    -DENABLE_yarpmod_portaudioRecorder=ON \
    -DENABLE_yarpmod_fakeAnalogSensor=ON \
    -DENABLE_yarpmod_fakeBattery=ON  \
    -DENABLE_yarpmod_fakeDepthCamera=ON \
    -DENABLE_yarpmod_fakeIMU=ON  \
    -DENABLE_yarpmod_fakeLaser=ON  \
    -DENABLE_yarpmod_fakeLocalizer=ON \
    -DENABLE_yarpmod_fakeMicrophone=ON  \
    -DENABLE_yarpmod_fakeMotionControl=ON  \
    -DENABLE_yarpmod_fakeNavigation=ON  \
    -DENABLE_yarpmod_fakeSpeaker=ON  \
    -DENABLE_yarpmod_fakebot=ON \
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
ENV YARP_DIR=$YARP_ROOT/build

#build bindings
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN cd yarp && cd bindings &&  mkdir build && cd build && cmake .. -DCREATE_PYTHON=ON && make -j11

# Install yarp completion
RUN sudo ln -s /home/user1/robotology/yarp/build/share/bash-completion/completions/yarp /usr/share/bash-completion/completions

# Build icub-main
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-main.git -b devel
RUN cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DENABLE_icubmod_gazecontrollerclient=ON && \
    make -j11
ENV ICUB_ROOT=$robotology_install_folder/icub-main
ENV ICUB_DIR=$ICUB_ROOT/build

# Build robots-configuration devel
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/robots-configuration.git -b master

# Install glpk
RUN sudo apt-get install -y glpk-doc glpk-utils libglpk-dev libglpk40
ENV GLPK_INCLUDE_DIR=/usr/include

# Build navigation
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/navigation.git -b master
RUN cd navigation && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DNAVIGATION_USE_NWC=ON && make -j11
ENV navigation_ROOT=$robotology_install_folder/navigation
ENV navigation_DIR=$navigation_ROOT/build

# CER stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer.git -b devel
RUN cd cer && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DENABLE_cermod_cerDoubleLidar=ON -DENABLE_cermod_cerOdometry=ON -DENABLE_cermod_tripodMotionControl=ON && make -j11
ENV CER_ROOT=$robotology_install_folder/cer
ENV CER_DIR=$CER_ROOT/build

# Build icub-contrib-common
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-contrib-common.git
RUN cd icub-contrib-common && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$robotology_install_folder/iCubContrib && sudo make -j11 install
ENV ICUBcontrib_DIR=$robotology_install_folder/iCubContrib/lib

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
WORKDIR $user_home
RUN git clone https://github.com/MariaLombardi/mutual-gaze-classifier-demo.git
RUN cd mutual-gaze-classifier-demo && \
    mkdir build && \
    cd ./build && \
    cmake .. && \
    make && sudo make install

USER $robotology_install_user
# Set up .bashrc
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/$robotology_install_user/.bashrc
ENV PATH=$PATH:/usr/local/cuda/bin/:$YARP_DIR/bin:$navigation_DIR/bin:$CER_DIR/bin:$ICUB_DIR/bin:$robotology_install_folder/iCubContrib/bin:/home/user1/robotology/Groot/build:/home/user1/tour-guide-robot/build/bin

# Set environmental variables
ENV DISPLAY=:1
ENV YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUB_DIR/share/iCub:$navigation_DIR/share/navigation:$user_home/tour-guide-robot/build/share/behavior-tour-robot:$user_home/tour-guide-robot/build/share/yarp:$CER_DIR/share/CER:$robotology_install_folder/iCubContrib/share/ICUBcontrib
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64/:$robotology_install_folder/yarp/build/lib/
ENV PYTHONPATH=$PYTHONPATH:/home/user1/robotology/yarp/bindings/build/lib/python3/
ENV YARP_COLORED_OUTPUT=1
ENV QT_X11_NO_MITSHM=1

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

# Launch bash from $robotology_install_folder
USER $robotology_install_user
RUN sudo chown -R $robotology_install_user:$robotology_install_user /opt/ros && sudo chown -R $robotology_install_user:$robotology_install_user /home/$robotology_install_user/.ros

# Groot and BT dependencies
RUN sudo apt-get install -y libgtest-dev libzmq3-dev libboost-dev qtbase5-dev libqt5svg5-dev libdw-dev && cd /usr/src/gtest && sudo cmake CMakeLists.txt && sudo make
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP && cd BehaviorTree.CPP && git checkout 4b21907 && mkdir build && cd build && cmake .. && sudo make -j11
ENV BehaviorTreeV3_DIR=/home/user1/robotology/BehaviorTree.CPP/build/
WORKDIR $robotology_install_folder
RUN git clone https://github.com/BehaviorTree/Groot.git
RUN cd Groot && git submodule update --init --recursive && mkdir build && cd build && cmake .. && sudo make -j11

# Navigation stuff
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN /bin/bash -c 'source ~/.bashrc'
WORKDIR /home/user1/catkin_ws/src
RUN git clone https://github.com/rst-tu-dortmund/costmap_prohibition_layer
RUN git clone https://github.com/DatSpace/ros-navigation
RUN git clone https://github.com/DatSpace/ros-speckle-filter
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make install -DCMAKE_BUILD_TYPE=Release'
WORKDIR $user_home
RUN git clone https://github.com/dpirch/libfvad && cd libfvad && autoreconf -i && ./configure && make && sudo make install
RUN git clone https://github.com/elandini84/yarp-web-teleop.git
RUN git clone https://github.com/hsp-iit/tour-guide-robot -b ros1
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd tour-guide-robot && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DBTState=ON -DENABLE_faceExpressionImage=ON && sudo make -j11'

# GOOGLE
RUN git clone --recursive -b v1.43.2 https://github.com/grpc/grpc.git && cd grpc && mkdir build && \
    cd build && cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release \
    -DgRPC_SSL_PROVIDER=package -DgRPC_ZLIB_PROVIDER=package && sudo make -j11 install && sudo ldconfig
RUN cd && git clone https://github.com/googleapis/googleapis.git && cd googleapis && git checkout f78745822aadedb3c165d51372732d672b634db2 && sudo make -j11
RUN cd && git clone https://github.com/randaz81/speech.git -b gs_external_stream && cd speech && mkdir build && cd build && \
    cmake .. -DBUILD_GOOGLE_SPEECH=ON -Dgoogleapis_INCLUDE_DIR=/home/user1/googleapis/gens && sudo make -j11 install
ENV GOOGLE_APPLICATION_CREDENTIALS=/home/user1/hsp_google.json

# R1 NETWORK CONFIG
RUN echo "source ~/.bashrc_local" >> ~/.bashrc

# Cleanup
RUN sudo apt-get clean && sudo apt-get update
WORKDIR /home/user1/tour-guide-robot
RUN sudo updatedb

CMD ["bash"]
