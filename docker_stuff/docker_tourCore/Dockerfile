ARG base_img
FROM $base_img
LABEL maintainer="Ettore Landini"

ARG ros_distro
ARG yarp_branch="master"
ARG ros2_dev_remote="robotology"
ARG ros2_dev_branch=${yarp_branch}
ARG gazebo_pkg=ros-${ros_distro}-ros-gz
ARG battery="OFF"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive \
    DOCKER_UPDATED_ON=06_06_2024

# Install essentials
RUN apt-get update && apt-get install --fix-missing -y apt-utils software-properties-common sudo psmisc lsb-release protobuf-compiler libatlas-base-dev \
    tmux nano geany vim wget curl build-essential git gitk cmake cmake-curses-gui autoconf xserver-xorg-video-dummy xserver-xorg-legacy \
    net-tools terminator libjpeg-dev ffmpeg apt-transport-https ca-certificates gnupg libace-dev ycm-cmake-modules locales \
    python3-setuptools python3-pip iproute2 python3-tornado python-dev-is-python3 lsof iftop iputils-ping gdb bash-completion btop pipx && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./common/xorg.conf /etc/X11/xorg.conf

# ROS2 Installation
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get upgrade -y && apt-get install --fix-missing -y ros-${ros_distro}-desktop ros-${ros_distro}-navigation2 ros-${ros_distro}-nav2-bringup \
    ros-${ros_distro}-slam-toolbox ros-${ros_distro}-test-msgs ${gazebo_pkg} \
    python3-colcon-common-extensions python3-flake8 python3-flake8-blind-except python3-flake8-builtins \
    python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings \
    python3-flake8-import-order python3-flake8-quotes python3-pytest python3-pytest-cov python3-pytest-repeat \
    python3-pytest-rerunfailures python3-rosdep python3-vcstool python3-venv && rosdep init && rm -rf /var/lib/apt/lists/*

# Install yarp dependencies
RUN apt-get update && apt-get install --fix-missing -y libode8 libace-dev libboost-filesystem-dev libboost-system-dev \
    libboost-thread-dev libc6 python3 libgsl0-dev libncurses5-dev libsdl1.2-dev subversion git gfortran \
    libxmu-dev libode-dev wget unzip qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5svg5 \
    libqt5opengl5-dev libopencv-dev freeglut3-dev libtinyxml-dev libblas-dev coinor-libipopt-dev \
    liblapack-dev libmumps-dev qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls \
    qml-module-qt-labs-settings libqcustomplot-dev qml-module-qt-labs-folderlistmodel libedit-dev \
    libeigen3-dev libjpeg-dev libsimbody-dev libxml2-dev libjs-underscore portaudio19-dev libgtkmm-2.4-dev \
    libi2c-dev libv4l-dev libsoxr-dev

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
RUN sudo mkdir /usr/local/src/robot
ENV robotology_install_user=user1
ENV user1_home=/home/${robotology_install_user} \
    robotology_install_folder=/usr/local/src/robot
RUN sudo chown -R ${robotology_install_user}:${robotology_install_user} ${robotology_install_folder}

RUN pip3 install --break-system-packages numpy bpytop

# SWIG 4.2.1 - build from source as root to ensure compatibility with target architecture
USER root
WORKDIR ${user1_home}

# 1) install build deps and download sources
RUN mkdir -p Downloads && cd Downloads && \
    apt-get update && apt-get install --no-install-recommends -y autoconf automake libtool libtool-bin m4 flex bison libpcre3-dev pkg-config gettext build-essential wget ca-certificates && \
    wget -qO swig-4.2.1.tar.gz https://github.com/swig/swig/archive/refs/tags/v4.2.1.tar.gz && \
    tar xzf swig-4.2.1.tar.gz

# 2) run autogen in source tree
RUN cd ${user1_home}/Downloads/swig-4.2.1 && ./autogen.sh || true

# 3) configure, build, install and cleanup
RUN cd ${user1_home}/Downloads/swig-4.2.1 && ./configure --prefix=/usr/ && make -j$(nproc) && make install && \
    rm -rf /home/${robotology_install_user}/Downloads/swig-4.2.1* && rm -rf /var/lib/apt/lists/*

USER ${robotology_install_user}


# Build ycm
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/ycm.git -b master && \
    cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release && \
    make -j11
ENV YCM_DIR=${robotology_install_folder}/ycm/build

# Build YARP
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/yarp.git -b ${yarp_branch} && \
    cd yarp && mkdir build && cd build && \
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
ENV YARP_ROOT=${robotology_install_folder}/yarp \
    YARP_DIR=${robotology_install_folder}/yarp/build
ENV PATH=$PATH:$YARP_DIR/bin

# YARP bindings
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN cd yarp && cd bindings &&  mkdir build && cd build && \
    cmake .. -DCREATE_PYTHON=ON -DSWIG_EXECUTABLE=/usr/bin/swig -DSWIG_DIR=/usr/share/swig/4.2.1 && make -j11

# Install YARP completion
RUN sudo ln -s ${YARP_DIR}/share/bash-completion/completions/yarp /usr/share/bash-completion/completions

# Build icub-firmware-shared
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/icub-firmware-shared -b devel && \
    cd icub-firmware-shared && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j11
ENV icub_firmware_shared_ROOT=${robotology_install_folder}/icub-firmware-shared \
    icub_firmware_shared_DIR=${robotology_install_folder}/icub-firmware-shared/build

# Build icub-main
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/icub-main.git -b devel && \
    cd icub-main && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DALLOW_DEVICE_PARAM_PARSER_GENERATION=ON \
    -DENABLE_icubmod_cartesiancontrollerserver=ON \
    -DENABLE_icubmod_cartesiancontrollerclient=ON \
    -DENABLE_icubmod_bcbBattery=${battery} \
    -DENABLE_icubmod_bmsBattery=${battery} \
    -DENABLE_icubmod_embObjBattery=${battery} \
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
ENV ICUB_ROOT=${robotology_install_folder}/icub-main \
    ICUB_DIR=${robotology_install_folder}/icub-main/build

# Build robots-configuration devel
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/robots-configuration.git -b master

# Build navigation
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/navigation.git -b master && \
    cd navigation && mkdir build && cd build && /bin/bash -c "source /opt/ros/${ros_distro}/setup.bash; cmake .. -DCMAKE_BUILD_TYPE=Release -DNAVIGATION_USE_NWC=ON -DNAVIGATION_USE_ROS2=ON" && make -j11
ENV navigation_ROOT=${robotology_install_folder}/navigation \
    navigation_DIR=${robotology_install_folder}/navigation/build

# CER
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/cer.git && \
    cd cer && mkdir build && cd build && cmake .. \
                -DCMAKE_BUILD_TYPE=Release \
                -DENABLE_cermod_cerDoubleLidar=ON \
                -DENABLE_cermod_cerOdometry=ON \
                -DENABLE_cermod_tripodMotionControl=ON && make -j11
ENV CER_ROOT=${robotology_install_folder}/cer \
    CER_DIR=${robotology_install_folder}/cer/build

# CER-SIM
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/cer-sim -b ros2 && \
    cd cer-sim/colcon_ws && /bin/bash -c "source /opt/ros/${ros_distro}/setup.bash; colcon build; source ${robotology_install_folder}/cer-sim/colcon_ws/install/local_setup.bash"

# Ergocub software
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/icub-tech-iit/ergocub-software && \
    cd ergocub-software && \
    cmake -S . -B build -DENABLE_couplingXCubHandMk5=ON -DCOMPILE_ergoCubModels=ON && cmake --build build
ENV ERGOCUB_SOFTWARE_DIR=${robotology_install_folder}/ergocub-software/build

# r1-models
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN mkdir installed && \
    git clone https://github.com/icub-tech-iit/r1-models && \
    cd r1-models && cmake -S . -B build -DCMAKE_INSTALL_PREFIX=${robotology_install_user}/installed && \
    cmake --build build && cmake --build build --target install

# YARP-DEVICES-ROS2
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/${ros2_dev_remote}/yarp-devices-ros2 -b ${ros2_dev_branch} && \
    cd yarp-devices-ros2/ros2_interfaces_ws && /bin/bash -c "source /opt/ros/${ros_distro}/setup.bash; colcon build" && \
    cd .. && mkdir build && cd build && /bin/bash -c "source /opt/ros/${ros_distro}/setup.bash; source ${robotology_install_folder}/yarp-devices-ros2/ros2_interfaces_ws/install/setup.bash; cmake .. -DYARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs=ON -DYARP_ROS2_USE_SYSTEM_yarp_control_msgs=ON; make -j11"

# YARP RPLidar Devices
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/yarp-device-rplidar -b master && \
    cd yarp-device-rplidar && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_rpLidar4=OFF && make -j11
ENV yarp-device-rpLidar_ROOT=${robotology_install_folder}/yarp-device-rplidar \
    yarp-device-rpLidar_DIR=${robotology_install_folder}/yarp-device-rplidar/build

# Build icub-contrib-common
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/robotology/icub-contrib-common.git && \
    cd icub-contrib-common && mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=${robotology_install_folder}/iCubContrib && make -j11 install
ENV ICUBcontrib_DIR=${robotology_install_folder}/iCubContrib/lib \
    ICUBcontrib_SHARE=${robotology_install_folder}/iCubContrib/share/ICUBcontrib

# Set environmental variables
USER ${robotology_install_user}
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/${robotology_install_user}/.bashrc
ENV TOUR_GUIDE_ROBOT_SOURCE_DIR=${robotology_install_folder}/tour-guide-robot \
    PATH=$PATH:${YARP_DIR}/bin:${navigation_DIR}/bin:${CER_DIR}/bin:${ICUB_DIR}/bin:${ICUB_DIR}:${robotology_install_folder}/iCubContrib/bin:${robotology_install_folder}/Groot/build:${robotology_install_folder}/tour-guide-robot/build/bin:${robotology_install_folder}/icub-main/build:${robotology_install_folder}/tour-guide-robot/app/navigation2/scripts:${robotology_install_folder}/tour-guide-robot/app/headSynchronizer/scripts \
    ROBOT_CODE=${robotology_install_folder} \
    CER_SIM_ROOT_DIR=${robotology_install_folder}/cer-sim \
    DISPLAY=:1 \
    LD_LIBRARY_PATH=${robotology_install_folder}/yarp/build/lib/yarp/ \
    YARP_COLORED_OUTPUT=1 \
    QT_X11_NO_MITSHM=1 \
    PYTHONPATH=$PYTHONPATH:${robotology_install_folder}/yarp/bindings/build/lib/python3/ \
    CYCLONEDDS_URI=${user1_home}/.config/cyclone_dds_settings.xml

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

# Launch bash from ${robotology_install_folder}
USER ${robotology_install_user}
RUN sudo chown -R ${robotology_install_user}:${robotology_install_user} /opt/ros && sudo chown -R ${robotology_install_user}:${robotology_install_user} /home/${robotology_install_user}/.ros

# Groot and BT dependencies
RUN sudo apt-get update && sudo apt-get install --fix-missing -y libgtest-dev libzmq3-dev libboost-dev qtbase5-dev libqt5svg5-dev libdw-dev && \
cd /usr/src/gtest && sudo cmake CMakeLists.txt && sudo make && sudo rm -rf /var/lib/apt/lists/*
USER ${robotology_install_user}
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/BehaviorTree/Groot.git && \
#RUN cd Groot && git submodule update --init --recursive && mkdir build && cd build && cmake .. && sudo make -j11
    git clone https://github.com/elandini84/libfvad && cd libfvad && autoreconf -i && ./configure && sudo make install

# Navigation stuff
WORKDIR ${robotology_install_folder}
RUN git clone https://github.com/elandini84/yarp-web-teleop.git && cd yarp-web-teleop  && mkdir build && cd build && cmake .. && \
    cd ${robotology_install_folder} && git clone https://github.com/hsp-iit/tour-guide-robot -b ${ros_distro} && \
    /bin/bash -c "export ONNX_PATH=${robotology_install_folder}/onnxruntime-linux-x64-1.20.1; source /opt/ros/${ros_distro}/setup.bash; cd tour-guide-robot; mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release -DBTState=OFF -DENABLE_faceExpressionImage=ON; make -j11"

ENV WEB_TELEOP_DIR=${robotology_install_folder}/yarp-web-teleop/build \
    YARP_DATA_DIRS=${YARP_DIR}/share/yarp:${ICUB_DIR}/share/iCub:${navigation_DIR}/share/navigation:${robotology_install_folder}/tour-guide-robot/build/share/tour-guide-robot:${CER_DIR}/share/CER:${ICUBcontrib_DIR}/share/ICUBcontrib:${robotology_install_folder}/yarp-devices-ros2/build/share/yarp:${robotology_install_folder}/yarp-devices-ros2/build/share/yarp-devices-ros2:${robotology_install_folder}/yarp-device-rplidar/build/share/yarp:${ICUBcontrib_SHARE}:${robotology_install_folder}/yarp-web-teleop/share/yarp-web-teleop

# R1 NETWORK CONFIG
RUN yarp conf 192.168.100.10 10000

# Possibly temporary ROS2 network middleware change
RUN sudo apt-get update && sudo apt-get install --fix-missing -y cyclonedds-dev cyclonedds-tools \
ros-${ros_distro}-cyclonedds-dbgsym ros-${ros_distro}-cyclonedds ros-${ros_distro}-rmw-cyclonedds-cpp \
ros-${ros_distro}-rmw-cyclonedds-cpp-dbgsym libcycloneddsidl0 && \
sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "#!/bin/bash" >> /home/user1/.entrypoint.sh && \
    echo ". /opt/ros/${ros_distro}/setup.bash" >> /home/user1/.entrypoint.sh && \
    echo ". ${robotology_install_folder}/yarp-devices-ros2/ros2_interfaces_ws/install/setup.bash" >> /home/user1/.entrypoint.sh && \
    echo ". ${robotology_install_folder}/tour-guide-robot/app/navigation2/scripts/evaluate_ip.sh" >> /home/user1/.entrypoint.sh && \
    echo 'exec "$@"' >> /home/user1/.entrypoint.sh && \
    chmod +x /home/user1/.entrypoint.sh && \
    sed -i '/^case \$- in$/,/^esac$/s/^/#/' /home/user1/.bashrc

RUN echo "source /opt/ros/${ros_distro}/setup.bash" >> ${user1_home}/.bashrc
RUN echo "source ${robotology_install_folder}/yarp-devices-ros2/ros2_interfaces_ws/install/setup.bash" >> ${user1_home}/.bashrc

WORKDIR ${user1_home}
#RUN sudo updatedb
# Change the lidar devices permissions inside the docker to be able to start the devices with yarprobotinterface
ENTRYPOINT [ "/home/user1/.entrypoint.sh" ]
CMD if [ -e /dev/ttyUSB0 ]; then sudo chown user1:user1 /dev/ttyUSB0 && sudo chown user1:user1 /dev/ttyUSB1; fi && bash
