sudo xhost +
if [[ -n $1 ]]; then
    if [[ $1 == "-n"  || $1 == "--nvidia" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all elandini84/r1images:r1Sim2_cuda
    elif [[ $1 == "-u"  || $1 == "--ubuntu" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all elandini84/r1images:r1Sim2
    elif [[$1 == "--nogpu" ]]; then
        sudo docker run --rm -it --privileged --network host --pid host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 elandini84/r1images:r1Sim2
    elif [[ $1 == "-h"  || $1 == "--help" ]]; then
        echo "Syntax: ./start-docker-core [option]"
        echo "options:"
        echo "    -n, --nvidia    Starts the image with the r1Sim2_cuda tag"
        echo "    -u, --ubuntu    Starts the image with the r1Sim2 tag"
        echo "        --nogpu     Starts the image with the r1Sim2 tag and without gpu support"
        echo "    -h, --help      See current help"
        echo "If no option is passed, the r1Sim2 tag (with gpu support) is used"
    else
        echo "Wrong option."
        $0 -h
    fi
else
	echo "No option passed. Using the default tag"
    $0 -u
fi
