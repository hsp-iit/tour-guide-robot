#!/bin/bash

############################################################
# Help                                                     #
############################################################
usage()
{
    # Display Help
    echo "Syntax: ./build-docker.sh [options]"
    echo "options:"
    echo "    -b, --build                        Use the passed options to build a new image. If not passed, the options passed will be used to identify the image to run"
    echo "    -u, --ubuntu                       Build/run the image from the default ubuntu image ($UBUNTU_DEF) and with the '$UBUNTU_SUFFIX' tag"
    echo "    -qt                                Build/run the image from the qt6 image ($QT_DEF) and with the '$QT_SUFFIX' tag"
    echo "    -r, --ros-distro + \"ros_distro\"     Build/run the image with the passed ROS2 distro"
    echo "                                       If not passed, the '$ROS_DEF' one will be used"
    echo "        --sim                          Build/run the image with the 'sim' tag, to be used for simulation purposes (e.g. with Gazebo)"
    echo "    -s, --stable                       Build/run the image with the '$STABLE_SUFFIX' tag"
    echo "    -d, --devel                        Build/run the image with the '$DEVEL_SUFFIX' tag"
    echo "    -e, --repo  + \"repo_name\"          Build/run the image with the passed repository reference"
    echo "    -cv, --cuda-version + \"cuda_version\" Build/run the image with the passed cuda version"
    echo "                                       If not passed, the '$CUDA_VERSION' one will be used"
    echo "        --nogpu                        Run the image without gpu support (only for non nVidia based images)"
    echo "    -p, --print                        Just print the command that would be executed to build/run the image"
    echo "    -h, --help                         See current help"
    echo "If the parent image is not specified (neither -u nor -c), the '$UBUNTU_DEF' one will be used"
    echo "If the build type is not specified (neither -d nor -s), the '$DEVEL_SUFFIX' tag will be used"
    echo "If the ROS2 distro is not specified, 'humble' will be used."
    echo "If the repository is not specified, $REPO_DEF will be used."
    echo "WARNING: If a wrong ROS2 distro name is passed, the image build will fail"

    exit
}

version()
{
    echo "Current version: $VERSION"

    exit
}

get_opts()
{
    while [[ $# -gt 0 ]]
    do
        key="$1"
        case $key in
            -u|--ubuntu)
                if [[ $IMAGE_SET == "true" ]]; then
                    echo "Image type already set"
                    usage
                fi
                shift
                BASE_TAG=$UBUNTU_DEF
                BASE_REPO=$UBUNTU_REPO
                BASE_REPO_SET=true
                IMAGE_SET=true
                ;;
            -qt)
                if [[ $IMAGE_SET == "true" ]]; then
                    echo "Image type already set"
                    usage
                fi
                shift
                BASE_TAG=$QT_DEF
                BASE_REPO=$QT_REPO
                BASE_REPO_SET=true
                IMAGE_SET=true
                ;;
            --sim)
                if [[ $IMAGE_SET == "true" ]]; then
                    echo "Image type already set"
                    usage
                fi
                shift
                BASE_TAG=$SIM_DEF
                BASE_REPO=$UBUNTU_REPO
                BASE_REPO_SET=true
                IMAGE_SET=true
                ;;
            -s|--stable)
                if [[ $BUILD_SET == "true" ]]; then
                    echo "Build type already set"
                    usage
                fi
                shift
                BUILD_SUFFIX=$STABLE_SUFFIX
                BUILD_SET=true
                ;;
            -d|--devel)
                if [[ $BUILD_SET == "true" ]]; then
                    echo "Build type already set"
                    usage
                fi
                shift
                BUILD_SUFFIX=$DEVEL_SUFFIX
                BUILD_SET=true
                ;;
            -r|--ros_distro)
                if [[ $ROS_SET == "true" ]]; then
                    echo "ROS2 distro already set"
                    usage
                fi
                shift
                ROS_DISTRO=$1
                ROS_SET=true
                shift
                ;;
            -y|--yarp_vers)
                if [[ $YARP_SET == "true" ]]; then
                    echo "YARP branch already set already set"
                    usage
                fi
                shift
                YARP_BRANCH=$1
                YARP_SET=true
                shift
                ;;
            -e|--repository)
                if [[ $REPO_SET == "true" ]]; then
                    echo "Remote repository already set"
                    usage
                fi
                shift
                REPO=$1
                REPO_SET=true
                shift
                ;;
            -b|--build)
                shift
                GONNA_BUILD=true
                ;;
            --nogpu)
                shift
                RUN_WITH_GPU=false
                ;;
            -p|--print)
                shift
                JUST_PRINT=true
                ;;
            -h|--help)
                usage
                ;;
            -v|--version)
                version
                ;;
            -cv|--cuda-version)
                if [[ $CUDA_VERSION_SET == "true" ]]; then
                    echo "CUDA Version already set"
                    usage
                fi
                shift
                CUDA_VERSION=$1
                CUDA_VERSION_SET=true
                GONNA_BUILD=true
                shift
                ;;
            *)
                echo "Unsupported arg"
                usage
                ;;
        esac
    done
}

############################################################
############################################################
# Main program                                             #
############################################################
############################################################

# Set default values
. ../docker_mng_vars.sh
UBUNTU_DEF="tourCore2_ubuntu24.04"
SIM_DEF="tourSim2_ubuntu24.04"
UBUNTU_REPO="elandini84/r1images"
QT_DEF="tour_ubuntu_24.04_qt_6.8.3"
QT_REPO="ste93/convince"
#Set Variables
BUILD_SUFFIX=$DEVEL_SUFFIX
VERSION="1.0.0"
IMAGE_SET=false
BUILD_SET=false
GONNA_BUILD=false
RUN_WITH_GPU=true
ROS_DISTRO=$ROS_DEF
ROS_SET=false
BASE_REPO=$UBUNTU_REPO
BASE_REPO_SET=false
REPO=$REPO_DEF
REPO_SET=false
BASE_TAG=${UBUNTU_DEF}
CUDA_VERSION=12.8
CUDA_VERSION_SET=false

############################################################
# Process the input options. Add options as needed.        #
############################################################
# Get the options
get_opts $@

# if [[$IMAGE_SET == "true"]]; then
#     COMPLETE_IMAGE_NAME="$IMAGE$JUNCTION$cuda$CUDA_VERSION"
# else
#     COMPLETE_IMAGE_NAME=$REPO$REPO_SEP$BASE_TAG$JUNCTION$PARENT_SUFFIX$JUNCTION$ROS_DISTRO$JUNCTION$YARP_TAG$BUILD_SUFFIX
# fi

COMPLETE_IMAGE_NAME="${REPO}${REPO_SEP}${BASE_TAG}${JUNCTION}${ROS_DISTRO}${JUNCTION}cuda${JUNCTION}${CUDA_VERSION}${JUNCTION}${BUILD_SUFFIX}"
BASE_IMAGE_NAME="${BASE_REPO}${REPO_SEP}${BASE_TAG}${JUNCTION}${ROS_DISTRO}${JUNCTION}${BUILD_SUFFIX}"

if [[ $JUST_PRINT == "true" ]]; then
    if [[ $GONNA_BUILD == "true" ]]; then
        echo "docker build --build-arg base_img=$BASE_IMAGE_NAME --build-arg cuda_version=$CUDA_VERSION -t $COMPLETE_IMAGE_NAME ."
    else
        if [[ $RUN_WITH_GPU == "true" ]]; then
            echo "docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all $COMPLETE_IMAGE_NAME"
        elif [[ $RUN_WITH_GPU == "false" && $IMAGE == $UBUNTU_DEF ]]; then
            echo "docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 $COMPLETE_IMAGE_NAME"
        else
            echo "ERROR: You cannot run a nVidia based image without gpu support"
        fi
    fi
    exit
fi

if [[ $GONNA_BUILD == "true" ]]; then
    docker build --build-arg base_img=$BASE_IMAGE_NAME --build-arg cuda_version=$CUDA_VERSION -t $COMPLETE_IMAGE_NAME .
else
    xhost +
    if [[ $RUN_WITH_GPU == "true" ]]; then
        docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all $COMPLETE_IMAGE_NAME
    elif [[ $RUN_WITH_GPU == "false" && $IMAGE == $UBUNTU_DEF ]]; then
        docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 $COMPLETE_IMAGE_NAME
    else
        echo "ERROR: You cannot run a nVidia based image without gpu support"
    fi
fi
