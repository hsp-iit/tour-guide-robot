 #!/bin/bash

if [[ -n $1 ]]; then
    if [[ $1 == "-d"  || $1 == "--devel" ]]; then
        sudo docker build -t elandini84/r1images:tourCore2_devel .
    elif [[ $1 == "-s"  || $1 == "--stable" ]]; then
        sudo docker build -t elandini84/r1images:tourCore2_stable .
    elif [[ $1 == "-n" || $1 == "--no-nvidia" ]]; then
        sudo docker build -f DockerfileNoNvidia -t elandini84/r1images:tourCore2_noNvidia .
    elif [[ $1 == "-h"  || $1 == "--help" ]]; then
        echo "Syntax: ./build-docker.sh [option]"
        echo "options:"
        echo "    -d, --devel     Build the image with the tourCore2_devel tag"
        echo "    -s, --stable    Build the image with the tourCore2_stable tag"
        echo "    -n, --no-nvidia Build the image starting from ubuntu standard and not the nvidia image"
        echo "    -h, --help      See current help"
        echo "If no option is passed, the tourCore2_devel tag is used"
    else
        echo "Wrong option."
        $0 -h
    fi
else
	echo "No option passed. Using the default tag"
    $0 -d
fi
