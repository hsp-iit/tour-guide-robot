if [[ -n $1 ]]; then
    if [[ $1 == "-n"  || $1 == "--nvidia" ]]; then
        sudo docker build --build-arg base_img=nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04 -t elandini84/r1images:r1Sim2_cuda .
    elif [[ $1 == "-u"  || $1 == "--ubuntu" ]]; then
        sudo docker build --build-arg base_img=ubuntu:22.04 -t elandini84/r1images:r1Sim2 .
    elif [[ $1 == "-h"  || $1 == "--help" ]]; then
        echo "Syntax: ./build-docker.sh [option]"
        echo "options:"
        echo "    -n, --nvidia    Build the image from the nvidia cuda official image (nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04) and with the r1Sim2_cuda tag"
        echo "    -u, --ubuntu    Build the image from the default ubuntu image (ubuntu:22.04) and with the r1Sim2 tag"
        echo "    -h, --help      See current help"
        echo "If no option is passed, the ubuntu image will be used as a base (option -u)"
    else
        echo "Wrong option."
        $0 -h
    fi
else
	echo "No option passed. Using the default tag"
    $0 -u
fi
