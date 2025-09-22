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
    echo "    -c, --cuda                         Build/run the image from the nvidia cuda official image ($CUDA_DEF) and with the '$CUDA_SUFFIX' tag"
    echo "    -u, --ubuntu                       Build/run the image from the default ubuntu image ($UBUNTU_DEF) and with the '$UBUNTU_SUFFIX' tag"
    echo "    -s, --stable                       Build/run the image with the '$STABLE_SUFFIX' tag"
    echo "    -d, --devel                        Build/run the image with the '$DEVEL_SUFFIX' tag"
    echo "    -e, --repo  + \"repo_name\"          Build/run the image with the passed repository reference"
    echo "    -r, --ros_distro + \"distro_name\"   Build/run the image with the passed distro (the passed value will be also used to compose the image tag)"
    echo "    -g, --gz_version + \"gz_version\"    Build/run the image with the passed gazebo version (the passed value will be also used to compose the image tag)"
    echo "    -y, --yarp_branch + \"yarp branch\"  Build/run the image with the passed yarp branch (the passed value, if different from \"master\", will be also used to compose the image tag)."
    echo "                                       If not passed, the branch used will be \"$YARP_DEF\""
    echo "        --inner_cycl_dds               If passed, the cyclone dds configuration file in app/navigation2/conf will be used, otherwise the one in ~/.config will be used."
    echo "        --dds_conf_path + \"path\"     If passed, the cyclone dds configuration file will be copied from the specified path to the container. If not passed, the default one will be used."
    echo "        --nogpu                        If passed, the image will be run without gpu support (only for ubuntu images)"
    echo "    -v, --version                      Print the current version"
    echo "    -p, --print                        Just print the command that would be executed to build/run the image"
    echo "    -h, --help                         See current help"
    echo "If the parent image is not specified (neither -u nor -c), the '$UBUNTU_DEF' one will be used"
    echo "If the build type is not specified (neither -d nor -s), the '$DEVEL_SUFFIX' tag will be used"
    echo "If the ROS2 distro is not specified, $ROS_DEF will be used."
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
                IMAGE=$UBUNTU_DEF
                PARENT_SUFFIX=$UBUNTU_SUFFIX
                IMAGE_SET=true
                ;;
            -c|--cuda)
                if [[ $IMAGE_SET == "true" ]]; then
                    echo "Image type already set"
                    usage
                fi
                shift
                IMAGE=$CUDA_DEF
                PARENT_SUFFIX=$CUDA_SUFFIX
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
            -g|--gz_version)
                if [[ $GZ_SET == "true" ]]; then
                    echo "Gazebo version already set"
                    usage
                fi
                shift
                GZ_VERS=$1
                GZ_SET=true
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
            --innner_cycl_dds)
                shift
                CYCLON_CONF_PATH=$CYCLON_CONF_PATH_FBK
                ;;
            --dds_conf_path)
                shift
                if [[ -z $1 ]]; then
                    echo "You must specify a path for the cyclone dds configuration file"
                    usage
                fi
                CYCLON_CONF_PATH=$1
                if [[ ! -f $CYCLON_CONF_PATH ]]; then
                    echo "The cyclone dds configuration file does not exist in the specified path: $CYCLON_CONF_PATH"
                    echo "Using the default one: $CYCLON_CONF_PATH_DEF"
                    CYCLON_CONF_PATH=$CYCLON_CONF_PATH_DEF
                    if [[ ! -f $CYCLON_CONF_PATH ]]; then
                        echo "The default cyclone dds configuration file does not exist: $CYCLON_CONF_PATH"
                        echo "Using the fallback one in the repository: $CYCLON_CONF_PATH_FBK"
                        CYCLON_CONF_PATH=$CYCLON_CONF_PATH_FBK
                    fi
                else
                    echo "Using the cyclone dds configuration file in: $CYCLON_CONF_PATH"
                fi
                shift
                ;;
            -h|--help)
                usage
                ;;
            -p|--print)
                shift
                JUST_PRINT=true
                ;;
            -v|--version)
                version
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
source ../docker_mng_vars.sh
BASE_TAG_DEF="tourSim2"

#Set Variables
JUST_PRINT=false
BUILD_SUFFIX=$DEVEL_SUFFIX
VERSION="1.0.0"
ROS_DISTRO=$ROS_DEF
GZ_VERS=$GAZEBO_DEF
YARP_BRANCH=$YARP_DEF
PARENT_SUFFIX=$UBUNTU_SUFFIX
IMAGE_SET=false
BUILD_SET=false
GONNA_BUILD=false
RUN_WITH_GPU=true
ROS_SET=false
GZ_SET=false
YARP_SET=false
IMAGE=$UBUNTU_DEF
REPO=$REPO_DEF
REPO_SET=false
BASE_TAG=$BASE_TAG_DEF
CYCLON_CONF_PATH=$CYCLON_CONF_PATH_DEF

############################################################
# Process the input options. Add options as needed.        #
############################################################
# Get the options
get_opts $@

if [[ $YARP_BRANCH != "master" ]]; then
    YARP_TAG=$YARP_BRANCH$JUNCTION
fi
COMPLETE_IMAGE_NAME=$REPO$REPO_SEP$BASE_TAG$JUNCTION$PARENT_SUFFIX$JUNCTION$ROS_DISTRO$JUNCTION$YARP_TAG$BUILD_SUFFIX

if [[ $JUST_PRINT == "true" ]]; then
    if [[ $GONNA_BUILD == "true" ]]; then
        echo "sudo docker build --build-arg base_img=$IMAGE --build-arg gazebo_version $GZ_VERS --build-arg ros_distro=$ROS_DISTRO --build-arg yarp_branch=$YARP_BRANCH --build-arg ros2_dev_branch=$ROS2_DEV_BRANCH --build-arg ros2_dev_remote=$ROS2_DEV_REMOTE  -t $COMPLETE_IMAGE_NAME ."
    else
        if [[ $RUN_WITH_GPU == "true" ]]; then
            echo "sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all $COMPLETE_IMAGE_NAME"
        elif [[ $RUN_WITH_GPU == "false" ]]; then
            echo "sudo docker run --rm -it --privileged --network host --pid host -e DISPLAY -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 $COMPLETE_IMAGE_NAME bash"
        else
            echo "ERROR: You cannot run a nVidia based image without gpu support"
        fi
    fi
    exit
fi

if [[ $GONNA_BUILD == "true" ]]; then
    sudo docker build --build-arg base_img=$IMAGE --build-arg gazebo_version $GZ_VERS --build-arg ros_distro=$ROS_DISTRO --build-arg yarp_branch=$YARP_BRANCH --build-arg ros2_dev_branch=$ROS2_DEV_BRANCH --build-arg ros2_dev_remote=$ROS2_DEV_REMOTE  -t $COMPLETE_IMAGE_NAME .
else
    sudo xhost +
    if [[ $RUN_WITH_GPU == "true" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all $COMPLETE_IMAGE_NAME
    elif [[ $RUN_WITH_GPU == "false" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e DISPLAY -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 $COMPLETE_IMAGE_NAME bash
    else
        echo "ERROR: You cannot run a nVidia based image without gpu support"
    fi
fi
