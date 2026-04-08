ARG base_img="ubuntu:24.04"
FROM $base_img
LABEL maintainer="Ettore Landini"

ARG yarp_version="master"
ARG user_name="yarp-user"
ARG robot_code="/usr/local/src/robot"
ARG llm_branch="master"
ARG uid="1000"
ARG gid="1000"
ARG vcpkg_tag="tags/2025.01.13"
ARG tour_branch="jazzy"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive \
    DOCKER_UPDATED_ON=12_04_2024

# Install essentials
RUN apt-get update && apt-get install -y apt-utils \
    software-properties-common sudo psmisc lsb-release \
    protobuf-compiler libatlas-base-dev tmux nano geany \
    vim wget curl build-essential git gitk cmake cmake-curses-gui \
    autoconf xserver-xorg-video-dummy xserver-xorg-legacy \
    net-tools terminator libjpeg-dev ffmpeg apt-transport-https \
    ca-certificates gnupg libace-dev ycm-cmake-modules locales \
    curl zip unzip tar python3-setuptools python3-pip iproute2 \
    python3-tornado lsof iftop iputils-ping gdb bash-completion \
    btop libcurl4-openssl-dev && \
    rm -rf /var/lib/apt/lists/*

RUN sed -i 's/allowed_users=console/allowed_users=anybody/' /etc/X11/Xwrapper.config
COPY ./common/xorg.conf /etc/X11/xorg.conf

# Install yarp dependencies
RUN apt-get update && apt-get install -y libode8 libace-dev libboost-filesystem-dev libboost-system-dev \
    libboost-thread-dev libc6 python3 libgsl0-dev libncurses5-dev libsdl1.2-dev subversion git gfortran \
    libxmu-dev libode-dev wget unzip qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev libqt5svg5 \
    libqt5opengl5-dev libopencv-dev freeglut3-dev libtinyxml-dev libblas-dev coinor-libipopt-dev \
    liblapack-dev libmumps-dev qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls \
    qml-module-qt-labs-settings libqcustomplot-dev qml-module-qt-labs-folderlistmodel libedit-dev \
    libeigen3-dev libjpeg-dev libsimbody-dev libxml2-dev libjs-underscore portaudio19-dev libgtkmm-2.4-dev \
    libi2c-dev libv4l-dev libsqlite3-0 libsqlite3-dev libsoxr-dev libpulse-dev pulseaudio-utils alsa-utils

# Create user: ${user_name}
USER root
RUN useradd -l -u 33334 -G sudo -md /home/${user_name} -s /bin/bash -p ${user_name} ${user_name} && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Set ENV
USER ${user_name}
RUN sudo mkdir ${robot_code}
ENV robotology_install_user=${user_name} \
    yarp_user_home=/home/${user_name} \
    robot_code=${robot_code}
RUN sudo chown -R ${robotology_install_user}:${robotology_install_user} ${robot_code}

# Miniforge installation
USER ${robotology_install_user}
WORKDIR ${yarp_user_home}
RUN wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
RUN /bin/bash -c "bash ${yarp_user_home}/Miniforge3-$(uname)-$(uname -m).sh -b"
RUN ${yarp_user_home}/miniforge3/bin/conda init
RUN /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
    conda create -y -n openwakeword python=3.11 && \
    conda clean -afy"
RUN /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
    conda activate openwakeword && \
    pip install numpy bpytop openwakeword ipython && \
    conda clean -afy"

# Build ycm
USER ${robotology_install_user}
WORKDIR ${robot_code}
RUN git clone https://github.com/robotology/ycm.git -b master && \
    cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release && \
    make -j11
ENV YCM_DIR=${robot_code}/ycm/build

# Build YARP
USER ${robotology_install_user}
WORKDIR ${robot_code}
RUN git clone https://github.com/robotology/yarp.git -b $yarp_version && \
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
    make -j11 && cmake .. -DALLOW_IDL_GENERATION=ON && \
    make -j11 && cmake .. -DALLOW_IDL_GENERATION=OFF
ENV YARP_ROOT=${robot_code}/yarp \
    YARP_DIR=${robot_code}/yarp/build

# SWIG 4.21
WORKDIR ${yarp_user_home}
RUN sudo apt-get purge -y swig && mkdir Downloads && cd Downloads &&\
    wget -nv https://github.com/robotology/robotology-vcpkg-ports/releases/download/storage/swig_4_2_1_ubuntu_24_04.zip && \
    unzip -q swig_4_2_1_ubuntu_24_04.zip && \
    sudo mv swig_4_2_1_ubuntu_24_04_install/bin/* /usr/bin && \
    sudo mv swig_4_2_1_ubuntu_24_04_install/share/swig /usr/share

# YARP bindings
USER ${robotology_install_user}
WORKDIR ${robot_code}
RUN cd yarp && cd bindings &&  mkdir build && cd build && \
    cmake .. -DCREATE_PYTHON=ON -DSWIG_EXECUTABLE=/usr/bin/swig -DSWIG_DIR=/usr/share/swig/4.2.1 && make -j11

# Install YARP completion
RUN sudo ln -s /usr/local/share/bash-completion/completions/yarp /usr/share/bash-completion/completions

#CER app
RUN git clone --no-checkout --depth 1 --filter=blob:none https://github.com/robotology/cer && \
    cd cer && git sparse-checkout init --cone && git sparse-checkout set app && git checkout && \
    rm CMakeLists.txt && touch CMakeLists.txt && \
    echo "# Copyright: (C) 2025 HSP Istituto Italiano di Tecnologia" > CMakeLists.txt && \
    echo "# Authors: Ettore Landini" >> CMakeLists.txt && \
    echo "# CopyPolicy: Released under the terms of the GNU GPL v2.0." >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "# Prepare a device library called \"cermod\"." >> CMakeLists.txt && \
    echo "# A test executable called \"cermoddev\" will also be produced." >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "cmake_minimum_required(VERSION 3.5)" >> CMakeLists.txt && \
    echo "project(CER LANGUAGES CXX C VERSION 1.0.0)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set(VARS_PREFIX \"CER\")" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set(CMAKE_CXX_EXTENSIONS OFF)" >> CMakeLists.txt && \
    echo "set(CMAKE_CXX_STANDARD 11)" >> CMakeLists.txt && \
    echo "set(CMAKE_CXX_STANDARD_REQUIRED 11)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set_property(GLOBAL PROPERTY USE_FOLDERS ON)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "list(APPEND CMAKE_MODULE_PATH \"\${CMAKE_SOURCE_DIR}/cmake\")" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "find_package(YCM)" >> CMakeLists.txt && \
    echo "find_package(YARP 3.11.100 REQUIRED COMPONENTS os sig dev math idl_tools)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "find_package(IPOPT)" >> CMakeLists.txt && \
    echo "if(NOT IPOPT_FOUND)" >> CMakeLists.txt && \
    echo "    message(WARNING \"IPOPT library was not found, some kinematics module will not be available.\")" >> CMakeLists.txt && \
    echo "endif()" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "include(GNUInstallDirs)" >> CMakeLists.txt && \
    echo "set(CMAKE_RUNTIME_OUTPUT_DIRECTORY \"\${CMAKE_BINARY_DIR}/\${CMAKE_INSTALL_BINDIR}\")" >> CMakeLists.txt && \
    echo "set(CMAKE_LIBRARY_OUTPUT_DIRECTORY \"\${CMAKE_BINARY_DIR}/\${CMAKE_INSTALL_LIBDIR}\")" >> CMakeLists.txt && \
    echo "set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY \"\${CMAKE_BINARY_DIR}/\${CMAKE_INSTALL_LIBDIR}\")" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL \"\${PROJECT_NAME} is always built with dynamic plugins\")" >> CMakeLists.txt && \
    echo "yarp_configure_external_installation(\${PROJECT_NAME} WITH_PLUGINS)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "include(AddInstallRPATHSupport)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "add_install_rpath_support(LIB_DIRS \"\${CMAKE_INSTALL_FULL_LIBDIR}\"" >> CMakeLists.txt && \
    echo "                          BIN_DIRS \"\${CMAKE_INSTALL_FULL_BINDIR}\"" >> CMakeLists.txt && \
    echo "                                   \"\${CMAKE_INSTALL_PREFIX}/\${CER_DYNAMIC_PLUGINS_INSTALL_DIR}\"" >> CMakeLists.txt && \
    echo "                          USE_LINK_PATH)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "# Everything is configured now, we can start adding source code" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "add_subdirectory(app)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "include(AddUninstallTarget)" >> CMakeLists.txt && \
    cmake -S . -B build

#R1_OBJECT_RETRIEVAL app
RUN git clone --no-checkout --filter=blob:none https://github.com/hsp-iit/r1-object-retrieval && \
    cd r1-object-retrieval && git sparse-checkout init --cone && git sparse-checkout set app && git checkout && \
    rm CMakeLists.txt && touch CMakeLists.txt && \
    echo "# Copyright: (C) 2025 iCub Facility Istituto Italiano di Tecnologia" >> CMakeLists.txt && \
    echo "# Authors: Raffaele Colombo" >> CMakeLists.txt && \
    echo "# CopyPolicy: Released under the terms of the GNU GPL v2.0." >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "cmake_minimum_required(VERSION 3.12)" >> CMakeLists.txt && \
    echo "project(R1_OBJECT_RETRIEVAL LANGUAGES CXX C VERSION 1.0.0)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set(CMAKE_CXX_EXTENSIONS OFF)" >> CMakeLists.txt && \
    echo "set(CMAKE_CXX_STANDARD 11)" >> CMakeLists.txt && \
    echo "set(CMAKE_CXX_STANDARD_REQUIRED 11)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set_property(GLOBAL PROPERTY USE_FOLDERS ON)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "list(APPEND CMAKE_MODULE_PATH \"\${CMAKE_SOURCE_DIR}/cmake\")" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "find_package(YCM)" >> CMakeLists.txt && \
    echo "find_package(YARP 3.11 REQUIRED)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "include(GNUInstallDirs)" >> CMakeLists.txt && \
    echo "set(CMAKE_RUNTIME_OUTPUT_DIRECTORY \"\${CMAKE_BINARY_DIR}/\${CMAKE_INSTALL_BINDIR}\")" >> CMakeLists.txt && \
    echo "set(CMAKE_LIBRARY_OUTPUT_DIRECTORY \"\${CMAKE_BINARY_DIR}/\${CMAKE_INSTALL_LIBDIR}\")" >> CMakeLists.txt && \
    echo "set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY \"\${CMAKE_BINARY_DIR}/\${CMAKE_INSTALL_LIBDIR}\")" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL \"\${PROJECT_NAME} is always built with dynamic plugins\")" >> CMakeLists.txt && \
    echo "yarp_configure_external_installation(\${PROJECT_NAME} WITH_PLUGINS)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt && \
    echo "# Everything is configured now, we can start adding source code" >> CMakeLists.txt && \
    echo "add_subdirectory(app)" >> CMakeLists.txt && \
    echo "" >> CMakeLists.txt


# Set environmental variables
USER ${robotology_install_user}
RUN echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \$ '" >> /home/${robotology_install_user}/.bashrc
ENV PATH=$PATH:$YARP_DIR/bin \
    DISPLAY=:1 \
    YARP_DATA_DIRS=$YARP_DIR/share/yarp:${robot_code}/cer/build/share/CER:${robot_code}/r1-object-retrieval/build/share/R1_OBJECT_RETRIEVAL \
    LD_LIBRARY_PATH=${robot_code}/yarp/build/lib/yarp/ \
    YARP_COLORED_OUTPUT=1 \
    QT_X11_NO_MITSHM=1 \
    PYTHONPATH=$PYTHONPATH:${YARP_ROOT}/bindings/build/lib/python3/

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

USER ${user_name}
WORKDIR /home/${user_name}/

### Install google from vcpkg ############################################# START ####
WORKDIR /home/${user_name}/Downloads
RUN git clone https://github.com/Microsoft/vcpkg.git && \
    cd vcpkg && git checkout ${google_cloud_version} && ./bootstrap-vcpkg.sh -disableMetrics && \
    ./vcpkg install --clean-after-build google-cloud-cpp[texttospeech] google-cloud-cpp[speech] google-cloud-cpp[dialogflow-cx]
ENV VCPKG_ROOT=/home/${user_name}/Downloads/vcpkg
### Packaging google-cloud-cpp instructions ############################################### END ####

ENV GOOGLE_APPLICATION_CREDENTIALS=/home/${user_name}/.config/google-credential/hsp_google.json \
    BUILD_SUFFIX=build

WORKDIR ${robot_code}
RUN git clone https://github.com/robotology/yarp-devices-google-cloud && \
   cd yarp-devices-google-cloud && cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=/home/yarp-user/Downloads/vcpkg/scripts/buildsystems/vcpkg.cmake && \
   cmake --build build
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-devices-google-cloud/build/share/yarp:${robot_code}/yarp-devices-google-cloud/build/share/

RUN git clone https://github.com/robotology/yarp-devices-azure-openai && \
   cd yarp-devices-azure-openai && cmake -B build -S . && cmake --build build
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-devices-azure-openai/build/share/yarp:${robot_code}/yarp-devices-azure-openai/build/share/

WORKDIR /home/${user_name}/Downloads
# Install openai c++ dependencies
RUN cd vcpkg && ./vcpkg install nlohmann-json

    # Install openai c++ community library
RUN git clone https://github.com/D7EAD/liboai.git && \
    cd liboai && cd liboai && \
    cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=/home/${user_name}/Downloads/vcpkg/scripts/buildsystems/vcpkg.cmake \
    -DCMAKE_POSITION_INDEPENDENT_CODE=true && \
    cd build && make -j8 && sudo make install

# Configure openai credentials
RUN echo "set -a" >> /home/${user_name}/.bashrc && \
    echo "source ~/.env/config.env" >> /home/${user_name}/.bashrc && \
    echo "set +a" >> /home/${user_name}/.bashrc

# Install yarpGPT
WORKDIR ${robot_code}
RUN git clone https://github.com/robotology/yarp-devices-llm -b ${llm_branch} && \
    cd yarp-devices-llm && mkdir build && \
    cmake -B build -S . \
    -DCMAKE_TOOLCHAIN_FILE=/home/${user_name}/Downloads/vcpkg/scripts/buildsystems/vcpkg.cmake \
    -DLIBOAI_INSTALL_PATH=/home/${user_name}/Downloads/liboai && cd build && \
    make -j8
ENV YARP_DEVICES_LLM_DIR=${robot_code}/yarp-devices-llm/build \
    YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-devices-llm/build/share/yarp:${robot_code}/yarp-devices-llm/build/share \
    PATH=$PATH:${robot_code}/yarp-devices-llm/build/bin

# Set up ONNX Runtime
WORKDIR ${robot_code}
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-x64-1.20.1.tgz
RUN tar -xvzf onnxruntime-linux-x64-1.20.1.tgz
ENV ONNX_PATH=${robot_code}/onnxruntime-linux-x64-1.20.1

#SileroVAD
WORKDIR ${robot_code}
RUN git clone https://github.com/snakers4/silero-vad

RUN git clone https://github.com/hsp-iit/tour-guide-robot -b $tour_branch
RUN /bin/bash -c "export ONNX_PATH=${robot_code}/onnxruntime-linux-x64-1.20.1; cd tour-guide-robot; mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release -DBTState=OFF -DENABLE_faceExpressionImage=ON -DSILERO_VAD=ON; make -j11"

ENV TOUR_GUIDE_ROBOT_SOURCE_DIR=${robot_code}/tour-guide-robot \
    YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/tour-guide-robot/build/share/tour-guide-robot \
    PATH=${PATH}:${robot_code}/tour-guide-robot/build/bin

RUN echo "conda activate openwakeword" >> /home/${user_name}/.bashrc && \
    sed -i '/^case \$- in$/,/^esac$/s/^/#/' /home/${user_name}/.bashrc

WORKDIR ${yarp_user_home}
CMD bash && conda activate openwakeword
