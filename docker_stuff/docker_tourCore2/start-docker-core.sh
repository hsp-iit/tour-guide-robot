#!/bin/bash

sudo xhost +

if [[ -n $1 ]]; then
    if [[ $1 == "-d"  || $1 == "--devel" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/hosts:/etc/hosts -e QT_X11_NO_MITSHM=1 --gpus all elandini84/r1images:tourCore2_devel
    elif [[ $1 == "-s"  || $1 == "--stable" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/hosts:/etc/hosts -e QT_X11_NO_MITSHM=1 --gpus all elandini84/r1images:tourCore2_stable
    elif [[$1 == "-n" || $1 == "--no-nvidia"]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/hosts:/etc/hosts -e QT_X11_NO_MITSHM=1 elandini84/r1images:tourCore2_noNvidia
    elif [[ $1 == "-h"  || $1 == "--help" ]]; then
        echo "Syntax: ./start-docker-core [option]"
        echo "options:"
        echo "    -d, --devel     Starts the image with the tourCore2_devel tag"
        echo "    -s, --stable    Starts the image with the tourCore2_stable tag"
        echo "    -h, --help      See current help"
        echo "If no option is passed, the tourCore2_stable tag is used"
    else
        echo "Wrong option."
        $0 -h
    fi
else
	echo "No option passed. Using the default tag"
    $0 -s
fi
