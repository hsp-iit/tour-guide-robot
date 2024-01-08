ARG base_img
FROM $base_img
LABEL maintainer="Ettore Landini"

ARG ros_distro
ARG yarp_branch="master"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_UPDATED_ON=20_06_2023

# Install essentials
RUN apt-get update && apt-get install -y apt-utils software-properties-common sudo psmisc lsb-release protobuf-compiler libatlas-base-dev \
    tmux nano geany vim wget curl build-essential git gitk cmake cmake-curses-gui autoconf xserver-xorg-video-dummy xserver-xorg-legacy \
    net-tools terminator libjpeg-dev ffmpeg apt-transport-https ca-certificates gnupg libace-dev ycm-cmake-modules locales \
    python3-setuptools python3-pip iproute2 python3-tornado swig lsof iftop iputils-ping gdb bash-completion btop mlocate && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy bpytop

RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./common/xorg.conf /etc/X11/xorg.conf

# ROS2 Installation
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y && apt-get install -y ros-$ros_distro-desktop ros-$ros_distro-navigation2 ros-$ros_distro-nav2-bringup \
    ros-$ros_distro-slam-toolbox ros-$ros_distro-test-msgs ros-$ros_distro-gazebo-ros \
    python3-colcon-common-extensions python3-flake8 python3-flake8-blind-except python3-flake8-builtins \
    python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings \
    python3-flake8-import-order python3-flake8-quotes python3-pytest python3-pytest-cov python3-pytest-repeat \
    python3-pytest-rerunfailures python3-rosdep python3-vcstool && rosdep init && rm -rf /var/lib/apt/lists/*

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

# ROS2 Installation (part2)
USER user1
RUN sudo rosdep fix-permissions && rosdep update

# Set ENV
USER user1
RUN mkdir /home/user1/robotology
ENV robotology_install_user user1
ENV user1_home /home/$robotology_install_user
ENV robotology_install_folder $user1_home/robotology

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
RUN git clone https://github.com/robotology/yarp.git -b ${yarp_branch}
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
    -DENABLE_yarpmod_portaudio=ON \
    -DENABLE_yarpmod_portaudioPlayer=ON \
    -DENABLE_yarpmod_portaudioRecorder=ON \
    -DENABLE_yarpmod_fakeBattery=ON  \
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

# Build icub-firmware-shared
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-firmware-shared -b devel
RUN cd icub-firmware-shared && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j11
ENV icub_firmware_shared_ROOT=$robotology_install_folder/icub-firmware-shared
ENV icub_firmware_shared_DIR=$robotology_install_folder/icub-firmware-shared/build

# Build icub-main
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-main.git -b devel
RUN cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DENABLE_icubmod_embObjAnalogSensor=ON \
    -DENABLE_icubmod_embObjFTsensor=ON \
    -DENABLE_icubmod_embObjIMU=ON \
    -DENABLE_icubmod_embObjInertials=ON \
    -DENABLE_icubmod_embObjMais=ON \
    -DENABLE_icubmod_embObjMotionControl=ON \
    -DENABLE_icubmod_embObjMultiEnc=ON \
    -DENABLE_icubmod_embObjPSC=ON \
    -DENABLE_icubmod_embObjSkin=ON \
    -DENABLE_icubmod_embObjStrain=ON \
    -DENABLE_icubmod_parametricCalibrator=ON \
    -DENABLE_icubmod_parametricCalibratorEth=ON \
    -DICUBMAIN_COMPILE_SIMULATORS=OFF \
    -DENABLE_icubmod_gazecontrollerclient=ON && \
    make -j11
ENV ICUB_ROOT=$robotology_install_folder/icub-main
ENV ICUB_DIR=$robotology_install_folder/icub-main/build

# Build robots-configuration devel
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/robots-configuration.git -b devel

# Build navigation
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/navigation.git -b master
RUN cd navigation && mkdir build && cd build && /bin/bash -c "source /opt/ros/$ros_distro/setup.bash; cmake .. -DCMAKE_BUILD_TYPE=Release -DNAVIGATION_USE_NWC=ON -DNAVIGATION_USE_ROS2=ON" && make -j11
ENV navigation_ROOT=$robotology_install_folder/navigation
ENV navigation_DIR=$robotology_install_folder/navigation/build

# CER
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer.git -b devel
RUN cd cer && mkdir build && cd build && cmake .. \
                -DCMAKE_BUILD_TYPE=Release \
                -DENABLE_cermod_cerDoubleLidar=ON \
                -DENABLE_cermod_cerOdometry=ON \
                -DENABLE_cermod_tripodMotionControl=ON && make -j11
ENV CER_ROOT=$robotology_install_folder/cer
ENV CER_DIR=$robotology_install_folder/cer/build

# CER-SIM
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/cer-sim -b ros2
RUN cd cer-sim/colcon_ws && /bin/bash -c "source /opt/ros/$ros_distro/setup.bash; colcon build; source /home/user1/robotology/cer-sim/colcon_ws/install/local_setup.bash"

# YARP-DEVICES-ROS2
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/yarp-devices-ros2 -b master
RUN cd yarp-devices-ros2/ros2_interfaces_ws && /bin/bash -c "source /opt/ros/$ros_distro/setup.bash; colcon build"
RUN cd yarp-devices-ros2 && mkdir build && cd build && /bin/bash -c "source /opt/ros/$ros_distro/setup.bash; source /home/user1/robotology/yarp-devices-ros2/ros2_interfaces_ws/install/setup.bash; cmake .. -DYARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs=ON -DYARP_ROS2_USE_SYSTEM_yarp_control_msgs=ON; make -j11"

# YARP RPLidar Devices
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/yarp-device-rplidar -b master
RUN cd yarp-device-rplidar && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_rpLidar4=OFF && make -j11
ENV yarp-device-rpLidar_ROOT=$robotology_install_folder/yarp-device-rplidar
ENV yarp-device-rpLidar_DIR=$robotology_install_folder/yarp-device-rplidar/build

# Build icub-contrib-common
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/robotology/icub-contrib-common.git
RUN cd icub-contrib-common && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$robotology_install_folder/iCubContrib && sudo make -j11 install
ENV ICUBcontrib_DIR=$robotology_install_folder/iCubContrib/lib
ENV ICUBcontrib_SHARE=$robotology_install_folder/iCubContrib/share/ICUBcontrib

# Set environmental variables
USER $robotology_install_user
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/$robotology_install_user/.bashrc
RUN echo "source /opt/ros/$ros_distro/setup.bash" >> /home/user1/.bashrc
RUN echo "source /home/user1/robotology/yarp-devices-ros2/ros2_interfaces_ws/install/local_setup.bash" >> /home/user1/.bashrc
ENV PATH=$PATH:$YARP_DIR/bin:$navigation_DIR/bin:$CER_DIR/bin:$ICUB_DIR/bin:$ICUB_DIR:$robotology_install_folder/iCubContrib/bin:/home/user1/robotology/Groot/build:/home/user1/tour-guide-robot/build/bin:/home/user1/robotology/icub-main/build
ENV DISPLAY=:1
ENV YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUB_DIR/share/iCub:$navigation_DIR/share/navigation:$user1_home/tour-guide-robot/build/share/tour-guide-robot:$CER_DIR/share/CER:$ICUBcontrib_DIR/share/ICUBcontrib:$robotology_install_folder/yarp-devices-ros2/build/share/yarp:$robotology_install_folder/yarp-devices-ros2/build/share/yarp-devices-ros2:$robotology_install_folder/yarp-device-rplidar/build/share/yarp:$ICUBcontrib_SHARE
ENV LD_LIBRARY_PATH=$robotology_install_folder/yarp/build/lib/yarp/
ENV YARP_COLORED_OUTPUT=1
ENV QT_X11_NO_MITSHM=1
ENV PYTHONPATH=$PYTHONPATH:/home/user1/robotology/yarp/bindings/build/lib/python3/
ENV CYCLONEDDS_URI=/home/user1/tour-guide-robot/app/navigation2/conf/cyclone_dds_settings.xml

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

# Launch bash from $robotology_install_folder
USER $robotology_install_user
RUN sudo chown -R $robotology_install_user:$robotology_install_user /opt/ros && sudo chown -R $robotology_install_user:$robotology_install_user /home/$robotology_install_user/.ros

# Groot and BT dependencies
RUN sudo apt-get update && sudo apt-get install -y libgtest-dev libzmq3-dev libboost-dev qtbase5-dev libqt5svg5-dev libdw-dev && \
    cd /usr/src/gtest && sudo cmake CMakeLists.txt && sudo make && sudo rm -rf /var/lib/apt/lists/*
USER $robotology_install_user
WORKDIR $robotology_install_folder
RUN git clone https://github.com/BehaviorTree/Groot.git
#RUN cd Groot && git submodule update --init --recursive && mkdir build && cd build && cmake .. && sudo make -j11
RUN git clone https://github.com/elandini84/libfvad && cd libfvad && autoreconf -i && ./configure && sudo make install

# Navigation stuff
WORKDIR ${user1_home}
RUN git clone https://github.com/elandini84/yarp-web-teleop.git
RUN git clone https://github.com/hsp-iit/tour-guide-robot -b $ros_distro
RUN /bin/bash -c "source /opt/ros/$ros_distro/setup.bash; cd tour-guide-robot; mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release -DBTState=ON -DENABLE_faceExpressionImage=ON; make -j11"

# GOOGLE
RUN git clone --recursive -b v1.51.0 https://github.com/grpc/grpc.git
RUN cd grpc && mkdir build && \
    cd build && cmake .. -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release \
    -DgRPC_SSL_PROVIDER=package -DgRPC_ZLIB_PROVIDER=package && sudo make -j11 install && sudo ldconfig
#RUN cd && git clone https://github.com/googleapis/googleapis.git && cd googleapis && git checkout f78745822aadedb3c165d51372732d672b634db2 && sudo make -j11
RUN cd && git clone https://github.com/googleapis/googleapis.git
RUN cd googleapis && sudo make -j11
RUN cd && git clone https://github.com/robotology/speech.git
RUN cd speech && mkdir build && cd build && \
    cmake .. -DBUILD_GOOGLE_SPEECH=ON -Dgoogleapis_INCLUDE_DIR=/home/user1/googleapis/gens && sudo make -j11 install
ENV GOOGLE_APPLICATION_CREDENTIALS=/home/user1/hsp_google.json

# R1 NETWORK CONFIG
RUN yarp conf 192.168.100.10 10000

# Possibly temporary ROS2 network middleware change
RUN sudo apt-get update && sudo apt-get install -y cyclonedds-dev cyclonedds-tools \
    ros-$ros_distro-cyclonedds-dbgsym ros-$ros_distro-cyclonedds ros-$ros_distro-rmw-cyclonedds-cpp \
    ros-$ros_distro-rmw-cyclonedds-cpp-dbgsym libcycloneddsidl0 && \
    sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "source /home/user1/tour-guide-robot/app/navigation2/scripts/evaluate_ip.sh" >> /home/user1/.bashrc
WORKDIR ${user1_home}/tour-guide-robot
RUN sudo updatedb
# Change the lidar devices permissions inside the docker to be able to start the devices with yarprobotinterface
CMD if [ -e /dev/ttyUSB0 ]; then sudo chown user1:user1 /dev/ttyUSB0 && sudo chown user1:user1 /dev/ttyUSB1; fi && bash
