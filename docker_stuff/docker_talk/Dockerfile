ARG base_img="ubuntu:24.04"
FROM $base_img
LABEL maintainer="Ettore Landini"

ARG yarp_version="master"
ARG user_name="yarp-user"
ARG robot_code="/usr/local/src/robot"
ARG llm_branch="master"
ARG uid="1000"
ARG gid="1000"
# ARG vcpkg_tag="tags/2025.01.13"
ARG tour_branch="jazzy"
ARG cuda_version="12.8"
ARG ai_model_path="/home/yarp-user/models"

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
    libi2c-dev libv4l-dev libsqlite3-0 libsqlite3-dev libsoxr-dev libpulse-dev pulseaudio-utils alsa-utils \
    meson

# Create user: ${user_name}
USER root
RUN useradd -l -u 33334 -G sudo -md /home/${user_name} -s /bin/bash -p ${user_name} ${user_name} && \
    # passwordless sudo for users in the 'sudo' group
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Set ENV
USER ${user_name}
ENV robotology_install_user=${user_name} \
    yarp_user_home=/home/${user_name} \
    robot_code=${robot_code}
RUN sudo mkdir ${robot_code} && mkdir ${yarp_user_home}/Downloads && \
    mkdir ${yarp_user_home}/userInstalled
RUN sudo chown -R ${robotology_install_user}:${robotology_install_user} ${robot_code}

### Install CUDA ############################################# START ####
WORKDIR ${yarp_user_home}/Downloads
RUN wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb
RUN sudo apt-get install ./libtinfo5_6.3-2ubuntu0.1_amd64.deb
RUN rm libtinfo5_6.3-2ubuntu0.1_amd64.deb

# Iterate over all the Ubuntu versions to find the latest CUDA version, starting from the most recent
RUN for ubuntu_v in 2404 2204 2004; do \
        # print the current CUDA version and Ubuntu version (useful if building with --progress=plain --no-cache option)
        echo "Looking for CUDA version "$cuda_version" in Ubuntu $ubuntu_v"; \
        echo "Looking into: https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$ubuntu_v/x86_64/"; \
        # extract cuda major and minor versions from the full version string
        CUDA_MAJOR=$(echo "$cuda_version" | cut -d. -f1) && CUDA_MINOR=$(echo "$cuda_version" | cut -d. -f2) && \
        # get the latest version of the specified major and minor versions
        CUDA_FILE=$(curl -s https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$ubuntu_v/x86_64/ | grep -oP "(?<=href=')[^']+" | grep "^cuda-$CUDA_MAJOR-$CUDA_MINOR" | tail -1); \
        if [ -n "$CUDA_FILE" ]; then \
            echo "Found CUDA file: $CUDA_FILE"; \
            # get the latest version of the keyring
            CUDA_KEYRING=$(curl -s https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$ubuntu_v/x86_64/ | grep -oP "(?<=href=')[^']+" | grep "^cuda-keyring" | tail -1); \
            # download the keyring
            wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu$ubuntu_v/x86_64/$CUDA_KEYRING; \
            break; \
        fi; \
    done; \
    # check if the CUDA version was found, otherwise exit
    if [ -z "$CUDA_FILE" ]; then \
        echo "CUDA version not found"; \
        exit 1; \
    fi

# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
RUN sudo dpkg -i cuda-keyring*.deb
RUN sudo apt-get update
# RUN sudo apt-get upgrade -y
RUN CUDA_MAJOR=$(echo "$cuda_version" | cut -d. -f1) && CUDA_MINOR=$(echo "$cuda_version" | cut -d. -f2) && \
    sudo apt-get install -y cuda-toolkit-$CUDA_MAJOR-$CUDA_MINOR

ENV PATH=/usr/local/cuda-$cuda_version/bin${PATH:+:${PATH}}
ENV LD_LIBRARY_PATH=/usr/local/cuda-$cuda_version/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

RUN rm cuda*
### Install CUDA ############################################# END ####

### Yarp related installations ############################### START ####
# Build ycm
USER ${robotology_install_user}
WORKDIR ${robot_code}
RUN git clone https://github.com/robotology/ycm.git -b master && \
    cd ycm && mkdir build && cd build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release && \
    make -j11
    ENV YCM_DIR=${robot_code}/ycm/build

# SWIG 4.21
WORKDIR ${yarp_user_home}
RUN sudo apt-get purge -y swig && cd Downloads &&\
    wget -nv https://github.com/robotology/robotology-vcpkg-ports/releases/download/storage/swig_4_2_1_ubuntu_24_04.zip && \
    unzip -q swig_4_2_1_ubuntu_24_04.zip && \
    sudo mv swig_4_2_1_ubuntu_24_04_install/bin/* /usr/bin && \
    sudo mv swig_4_2_1_ubuntu_24_04_install/share/swig /usr/share

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
    -DENABLE_yarppm_depthimage_to_rgb=ON \
    -DYARP_COMPILE_BINDINGS=ON \
    -DCREATE_PYTHON=ON -DSWIG_EXECUTABLE=/usr/bin/swig -DSWIG_DIR=/usr/share/swig/4.2.1 && \

    make -j11 && cmake .. -DALLOW_IDL_GENERATION=ON && \
    make -j11 && cmake .. -DALLOW_IDL_GENERATION=OFF
ENV YARP_ROOT=${robot_code}/yarp \
    YARP_DIR=${robot_code}/yarp/build


# YARP bindings
# USER ${robotology_install_user}
# WORKDIR ${robot_code}
# RUN cd yarp && cd bindings &&  mkdir build && cd build && \
#     cmake .. -DCREATE_PYTHON=ON -DSWIG_EXECUTABLE=/usr/bin/swig -DSWIG_DIR=/usr/share/swig/4.2.1 && make -j11

# Install YARP completion
RUN sudo ln -s /usr/local/share/bash-completion/completions/yarp /usr/share/bash-completion/completions

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
    PYTHONPATH=$PYTHONPATH:${YARP_DIR}/lib/python3/

# Manage yarp port
EXPOSE 10000/tcp 10000/udp

USER ${user_name}
WORKDIR /home/${user_name}/

# ### Install google from vcpkg ############################################# START ####
# WORKDIR /home/${user_name}/Downloads
# RUN git clone https://github.com/Microsoft/vcpkg.git && \
#     cd vcpkg && git checkout ${google_cloud_version} && ./bootstrap-vcpkg.sh -disableMetrics && \
#     ./vcpkg install --clean-after-build google-cloud-cpp[texttospeech] google-cloud-cpp[speech] google-cloud-cpp[dialogflow-cx]
# ENV VCPKG_ROOT=/home/${user_name}/Downloads/vcpkg
# ### Packaging google-cloud-cpp instructions ############################################### END ####

RUN git clone https://github.com/robotology/yarp-devices-azure-openai && \
cd yarp-devices-azure-openai && cmake -B build -S . && cmake --build build
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-devices-azure-openai/build/share/yarp:${robot_code}/yarp-devices-azure-openai/build/share/

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
ROBOT_CODE=${robot_code} \
PATH=${PATH}:${robot_code}/tour-guide-robot/build/bin

COPY ./environment.yml ${robot_code}/tour-guide-robot/docker_stuff/docker_talk/environment.yml

### Miniforge and Python environment ############################## START ####
USER ${robotology_install_user}
WORKDIR ${yarp_user_home}
RUN wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
RUN /bin/bash -c "bash ${yarp_user_home}/Miniforge3-$(uname)-$(uname -m).sh -b"
RUN ${yarp_user_home}/miniforge3/bin/conda init
RUN /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
conda env create -y -f ${TOUR_GUIDE_ROBOT_SOURCE_DIR}/docker_stuff/docker_talk/environment.yml && \
conda clean -afy"
### Miniforge and Python environment ############################## END ####

WORKDIR ${robot_code}

# Install openai c++ community library
RUN git clone https://github.com/D7EAD/liboai.git && \
    cd liboai && cd liboai && \
    /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
    conda activate unified-voice-pipeline && \
    cmake -B build -S . -DCMAKE_INSTALL_PREFIX=${yarp_user_home}/userInstalled \
    -DCMAKE_POSITION_INDEPENDENT_CODE=true && cd build && make -j$(nproc) && make install"

# Configure openai credentials
RUN echo "set -a" >> /home/${user_name}/.bashrc && \
echo "source ~/.env/config.env" >> /home/${user_name}/.bashrc && \
echo "set +a" >> /home/${user_name}/.bashrc

# Install yarpGPT
WORKDIR ${robot_code}
RUN git clone https://github.com/elandini84/yarp-devices-llm -b YARP4 && \
    cd yarp-devices-llm && mkdir build && \
    /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
    conda activate unified-voice-pipeline && \
    cmake -B build -S . -DCMAKE_INSTALL_PREFIX=${yarp_user_home}/userInstalled \
    -DCMAKE_POSITION_INDEPENDENT_CODE=true -DCMAKE_PREFIX_PATH=${yarp_user_home}/userInstalled && cd build && make -j$(nproc)"
ENV YARP_DEVICES_LLM_DIR=${robot_code}/yarp-devices-llm/build \
YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-devices-llm/build/share/yarp:${robot_code}/yarp-devices-llm/build/share \
PATH=$PATH:${robot_code}/yarp-devices-llm/build/bin

ENV GOOGLE_APPLICATION_CREDENTIALS=/home/${user_name}/.config/google-credential/hsp_google.json \
BUILD_SUFFIX=build

WORKDIR ${robot_code}
RUN git clone https://github.com/robotology/yarp-devices-google-cloud -b YARP4 && \
cd yarp-devices-google-cloud && \
/bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
conda activate unified-voice-pipeline && \
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DOPENSSL_ROOT_DIR=\${CONDA_PREFIX} && cmake --build build -- -j \$(nproc)"
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-devices-google-cloud/build/share/yarp:${robot_code}/yarp-devices-google-cloud/build/share/

# WebRTC audio processing repo (used for audio echo cancellation)
WORKDIR ${robot_code}
RUN git clone https://gitlab.freedesktop.org/pulseaudio/webrtc-audio-processing/ && \
    cd webrtc-audio-processing && meson . build -Dprefix=$PWD/install && ninja -C build && ninja -C build install

# AEC baseline that works based on WebRTC
WORKDIR ${robot_code}
RUN git clone https://github.com/hsp-iit/aec_baseline && \
    cd aec_baseline && mkdir build && cd build && export WEBRTC_AUDIO_PROCESSING_ROOT=${robot_code}/webrtc-audio-processing/install && \
    cmake .. && make -j4
ENV PATH=$PATH:${robot_code}/aec_baseline/build

### Install openWakeWord and voicebox python packages ############################## START ####
USER ${robotology_install_user}
WORKDIR ${yarp_user_home}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
RUN curl -fsSL https://bun.sh/install | bash
ENV BUN_INSTALL=${yarp_user_home}/.bun
ENV PATH=${BUN_INSTALL}/bin:${PATH}

RUN sudo apt update && sudo apt install -y just libwebkit2gtk-4.1-dev \
  file python3.12-venv \
  libxdo-dev nlohmann-json3-dev \
  libssl-dev \
  libayatana-appindicator3-dev \
  librsvg2-dev
WORKDIR ${robot_code}
RUN git clone https://github.com/dscripka/openWakeWord && cd openWakeWord && \
    /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
    conda activate unified-voice-pipeline && \
    pip install -e ."
RUN git clone https://github.com/jamiepine/voicebox.git && cd voicebox && \
    /bin/bash -c "source ${yarp_user_home}/miniforge3/etc/profile.d/conda.sh && \
    conda activate unified-voice-pipeline && \
    bun install"
WORKDIR ${robot_code}/voicebox

RUN /bin/bash -c 'set -euo pipefail; \
    source "${yarp_user_home}/miniforge3/etc/profile.d/conda.sh"; \
    conda activate unified-voice-pipeline; \
    setsid bun run dev:server & \
    server_pid=$!; \
    cleanup() { \
      kill -TERM -"$server_pid" 2>/dev/null || true; \
      sleep 2; \
      kill -KILL -"$server_pid" 2>/dev/null || true; \
      wait "$server_pid" 2>/dev/null || true; \
    }; \
    trap cleanup EXIT; \
    for i in $(seq 1 60); do \
      if curl -fsS http://localhost:17493/ >/dev/null 2>&1; then break; fi; \
      sleep 1; \
      if [ "$i" = 60 ]; then echo "voicebox server did not start"; exit 1; fi; \
    done; \
    curl -fsS -X POST http://localhost:17493/profiles \
      -H "Content-Type: application/json" \
      --max-time 30 \
      -d "{\"name\":\"Sarah_001\",\"description\":\"\",\"language\":\"en\",\"voice_type\":\"preset\",\"preset_engine\":\"kokoro\",\"preset_voice_id\":\"af_sarah\",\"design_prompt\":null,\"default_engine\":\"kokoro\"}"; \
    curl -fsS -X POST http://localhost:17493/profiles \
      -H "Content-Type: application/json" \
      --max-time 30 \
      -d "{\"name\":\"Sara_001\",\"description\":\"\",\"language\":\"it\",\"voice_type\":\"preset\",\"preset_engine\":\"kokoro\",\"preset_voice_id\":\"if_sara\",\"design_prompt\":null,\"default_engine\":\"kokoro\"}"; \
    cleanup; \
    if ss -ltnp | grep -q ":17493"; then \
      echo "ERROR: port 17493 still in use after cleanup"; \
      ss -ltnp | grep ":17493" || true; \
      exit 1; \
    fi'
### Install openWakeWord and voicebox python packages ############################## END ####

### Install whisper.cpp ############################### START ####
WORKDIR ${robot_code}
RUN git clone https://github.com/ggml-org/whisper.cpp
    # cd whisper.cpp && cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${yarp_user_home}/userInstalled \
    # -DGGML_CUDA=ON && \
    # cmake --build build -j8 && cmake --build build --target install
ENV whisper_DIR=${yarp_user_home}/userInstalled \
    LD_LIBRARY_PATH=${yarp_user_home}/userInstalled/lib:${LD_LIBRARY_PATH}
### Install whisper.cpp ############################### END ####

### Offline vocal interaction devices ################################ START ####
WORKDIR ${robot_code}
RUN git clone https://github.com/elandini84/yarp-device-speechTranscription-whisper -b refactor/general_update
    # cd yarp-device-speechTranscription-whisper && \
    # cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -Dwhisper_DIR=${yarp_user_home}/userInstalled && \
    # cmake --build build -j8
RUN git clone https://github.com/robotology/yarp-device-voicebox -b YARP4 && \
    cd yarp-device-voicebox && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j8
RUN git clone https://github.com/robotology/yarp-device-llama2 -b YARP4 && \
    cd yarp-device-llama2 && \
    ./setup_submodule.sh
    # cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DLLAMA_ALL_WARNINGS=ON \
    # -DGGML_CUDA=ON -DLLAMA_BUILD_COMMON=ON \
    # -DLLAMA_BUILD_EXAMPLES=ON -DLLAMA_BUILD_SERVER=ON && \
    # cmake --build build -j8
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-device-speechTranscription-whisper/build/share/yarp:${robot_code}/yarp-device-voicebox/build/share/yarp:${robot_code}/yarp-device-llama2/build/share/yarp
ENV TEST=1
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${ai_model_path}
ENV PATH=${PATH}:${robot_code}/yarp-device-speechTranscription-whisper/build/bin:${robot_code}/yarp-device-voicebox/build/bin:${robot_code}/yarp-device-llama2/build/bin
ENV VOICEBOX=${robot_code}/voicebox
### Offline vocal interaction devices ################################ END ####

### YARP MCP repos ############################################# START ####
WORKDIR ${robot_code}
RUN git clone https://github.com/hsp-iit/yarp-mcpServers.git && \
    git clone https://github.com/hsp-iit/yarp-mcpClient-generalPurpose.git
ENV YARP_DATA_DIRS=${YARP_DATA_DIRS}:${robot_code}/yarp-mcpClient-generalPurpose/resources:${robot_code}/yarp-mcpServers/resources
### YARP MCP repos ############################################# END ####

### Install ollama ################################ START ####
WORKDIR ${yarp_user_home}/Downloads
RUN sudo apt update && sudo apt install -y zstd
RUN curl -fsSL https://ollama.com/install.sh | sh
### Install ollama ################################ END ####

RUN echo "conda activate unified-voice-pipeline" >> /home/${user_name}/.bashrc && \
    sed -i '/^case \$- in$/,/^esac$/s/^/#/' /home/${user_name}/.bashrc

WORKDIR ${yarp_user_home}
CMD bash && conda activate unified-voice-pipeline
