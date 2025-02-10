# The dockerfile builds Ubuntu 24.04 with ROS2 Jazzy and custom CUDA Toolkit version (default is 12.1)

ARG base_img
FROM $base_img

ARG cuda_version
# # Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

RUN wget http://security.ubuntu.com/ubuntu/pool/universe/n/ncurses/libtinfo5_6.3-2ubuntu0.1_amd64.deb
RUN sudo apt-get install ./libtinfo5_6.3-2ubuntu0.1_amd64.deb
RUN rm libtinfo5_6.3-2ubuntu0.1_amd64.deb

# Iterate over all the Ubuntu versions to find the latest CUDA version, starting from the most recent
RUN for ubuntu_v in 2404 2204 2004 1804 1604 1404 1304 1204; do \
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