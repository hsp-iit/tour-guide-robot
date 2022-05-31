FROM ubuntu:20.04
LABEL maintainer="Konstantinos Karapas based on marco.randazzo@iit.it"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_UPDATED_ON=31_05_2022

# Install essentials
RUN apt-get update && apt-get install -y apt-utils software-properties-common sudo psmisc lsb-release \
    tmux nano geany vim tilde wget curl build-essential git gitk cmake cmake-curses-gui \
    libedit-dev libxml2-dev autoconf \
    net-tools terminator \
    libjpeg-dev ffmpeg apt-transport-https ca-certificates gnupg\
    epstool transfig tilde python3-setuptools python3-pip iproute2 python3-tornado swig
                   
RUN pip3 install numpy jupyter

# Install graphics
RUN apt-get update && apt-get install -y xfce4 xfce4-goodies xserver-xorg-video-dummy xserver-xorg-legacy

RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./common/xorg.conf /etc/X11/xorg.conf

# We need the most recent x11vnc
RUN sed -Ei 's/^# deb-src /deb-src /' /etc/apt/sources.list && apt-get update
RUN git clone https://github.com/LibVNC/x11vnc.git /opt/x11vnc && \
    cd /opt/x11vnc && \
    apt-get build-dep -y x11vnc && \
    autoreconf -fiv && \
    ./configure && \
    make -j11 && \
    rm /opt/x11vnc -Rf

# Install yarp dependencies
RUN echo "deb http://www.icub.org/ubuntu `lsb_release -cs` contrib/science" > /etc/apt/sources.list.d/icub.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 57A5ACB6110576A6
RUN apt-get update && apt-get install -y icub-common libopencv-dev

# Install Qt patch for Xfce
RUN apt-get install -y qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings

# other utilites
RUN apt-get install -y lsof iftop iputils-ping gdb bash-completion

# Select branch + build type
ARG BUILD_TYPE=Release

# ROS installation
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full  \
                   locate mercurial ros-noetic-teleop-tools ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard  \
                   ros-noetic-nav-core ros-noetic-map-server ros-noetic-gmapping ros-noetic-amcl ros-noetic-octomap-server ros-noetic-rtabmap-ros \
                   ros-noetic-rtabmap ros-noetic-mapviz ros-noetic-move-base python3-rosdep python3-rosinstall python3-rosinstall-generator \
                   python3-wstool ros-noetic-tf2-sensor-msgs ros-noetic-octomap-ros
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
ENV user1_home /home/$robotology_install_user
ENV robotology_install_folder $user1_home/robotology

#gazebo models
RUN mkdir -p $user1_home/.gazebo && git clone https://github.com/osrf/gazebo_models $user1_home/.gazebo/models

# Build ycm
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/ycm.git -b v0.14.1
RUN cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE && \
    make -j11

# Build yarp
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/yarp.git -b v3.7.0
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
ENV YARP_DIR=$robotology_install_folder/yarp/build

#build bindings
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN cd yarp && cd bindings &&  mkdir build && cd build && cmake .. -DCREATE_PYTHON=ON && make -j11

# Install yarp completion   
RUN sudo ln -s /usr/local/share/bash-completion/completions/yarp /usr/share/bash-completion/completions

# Build icub-main
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-main.git -b v1.25.0
RUN cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DENABLE_icubmod_gazecontrollerclient=ON && \
    make -j11
ENV ICUB_ROOT=$robotology_install_folder/icub-main
ENV ICUB_DIR=$robotology_install_folder/icub-main/build

# Build robots-configuration devel
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/robots-configuration.git -b v1.24.0

# Install glpk
RUN sudo apt-get install -y glpk-doc glpk-utils libglpk-dev libglpk40
ENV GLPK_INCLUDE_DIR=/usr/include

# Build navigation
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/navigation.git
RUN cd navigation && git checkout ab116badc195fbc2db71fe3974498664503483c7 && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DNAVIGATION_USE_NWC=ON && make -j11
ENV navigation_ROOT=$robotology_install_folder/navigation
ENV navigation_DIR=$robotology_install_folder/navigation/build

# gazebo plugins stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/gazebo-yarp-plugins -b v4.4.0
RUN cd gazebo-yarp-plugins && mkdir build && cd build && \
    cmake .. \ 
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \ 
    -DGAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS=ON && \
    make -j11
ENV GazeboYARPPlugins_ROOT=$robotology_install_folder/gazebo-yarp-plugins
ENV GazeboYARPPlugins_DIR=$robotology_install_folder/gazebo-yarp-plugins/build

# CER stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer.git
RUN cd cer && git checkout ba2c0fce70ba1596850eb3abcc5efa4ee56be329 && mkdir build && cd build && cmake .. \
                -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
                -DENABLE_cermod_cerDoubleLidar=ON \
                -DENABLE_cermod_cerOdometry=ON \
                -DENABLE_cermod_tripodMotionControl=ON && make -j11
ENV CER_ROOT=$robotology_install_folder/cer
ENV CER_DIR=$robotology_install_folder/cer/build

# CER-sim stuff
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer-sim && cd cer-sim && git checkout f95408abcdfd2b94221203140c81bde607adcdec

# Build icub-contrib-common
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-contrib-common.git
RUN cd icub-contrib-common && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$robotology_install_folder/iCubContrib && sudo make -j11 install
ENV ICUBcontrib_DIR=$robotology_install_folder/iCubContrib/build

USER $robotology_install_user

# Set up .bashrc
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/$robotology_install_user/.bashrc
ENV PATH=$PATH:/usr/local/cuda/bin/:$YARP_DIR/bin:$navigation_DIR/bin:$CER_DIR/bin:$ICUB_DIR/bin:$robotology_install_folder/iCubContrib/bin:/home/user1/robotology/Groot/build:/home/user1/tour-guide-robot/build/bin

# Set environmental variables
ENV DISPLAY=:1
ENV YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUB_DIR/share/iCub:$navigation_DIR/share/navigation:$user1_home/tour-guide-robot/build/share/behavior-tour-robot:$CER_DIR/share/CER:$ICUBcontrib_DIR/share/ICUBcontrib
ENV LD_LIBRARY_PATH=$robotology_install_folder/yarp/build/lib/yarp/
ENV GAZEBO_PLUGIN_PATH=$GazeboYARPPlugins_DIR/lib
ENV GAZEBO_MODEL_PATH=$robotology_install_folder/cer-sim/gazebo/
ENV YARP_COLORED_OUTPUT=1
ENV QT_X11_NO_MITSHM=1
ENV PYTHONPATH=$PYTHONPATH:/home/user1/robotology/yarp/bindings/build/lib/python3/

# Manage x11vnc port
EXPOSE 5901

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

# Launch bash from $robotology_install_folder
USER $robotology_install_user
RUN sudo chown -R $robotology_install_user:$robotology_install_user /opt/ros && sudo chown -R $robotology_install_user:$robotology_install_user /home/$robotology_install_user/.ros

# Konstantinos Karapas stuff only
RUN mkdir -p ~/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make'
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN /bin/bash -c 'source ~/.bashrc'
WORKDIR /home/user1/catkin_ws/src
RUN git clone https://github.com/rst-tu-dortmund/costmap_prohibition_layer
RUN git clone https://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@github.com/robotology-dependencies/r1-navigation
RUN git clone https://github.com/DatSpace/ros-speckle-filter
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd ~/catkin_ws; catkin_make install -DCMAKE_BUILD_TYPE=Release'
ENV YARP_CLOCK=/clock
WORKDIR /home/user1
RUN cd && git clone https://github.com/dpirch/libfvad && cd libfvad && autoreconf -i && ./configure && make && sudo make install
RUN cd && git clone https://github.com/elandini84/yarp-web-teleop.git
RUN git clone https://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@github.com/robotology-playground/tour-guide-robot -b master
RUN cd tour-guide-robot && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j11

# R1 NETWORK CONFIG
RUN echo "source ~/.bashrc_local" >> ~/.bashrc

#cleanup
RUN sudo apt-get clean && sudo apt-get update
WORKDIR /home/user1/tour-guide-robot
RUN sudo updatedb

CMD ["bash"]
