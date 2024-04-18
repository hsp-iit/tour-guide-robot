if [ -z "${CER_SIM_ROOT_DIR}" ]; then
    if [ -z "${ROBOT_CODE}" ]; then
        if [ -d "~/robotology/cer-sim" ]; then
            echo "Setting CER_SIM_ROOT_DIR to ~/robotology/cer-sim"
            echo "export CER_SIM_ROOT_DIR=~/robotology/cer-sim" >> ~/.bashrc
        else
            echo "Cer-sim directory not found"
            echo "If you have cer-sim cloned on your machine, please set the CER_SIM_ROOT_DIR manually."
            echo "Otherwise you can clone it by running the following command:"
            echo "git clone https://github.com/robotology/cer-sim.git in a directory of your choice and try again"
        fi
    elif [ -d "${ROBOT_CODE}/cer-sim" ]; then
        echo "Setting CER_SIM_ROOT_DIR to ${ROBOT_CODE}/cer-sim"
        echo "export CER_SIM_ROOT_DIR=${ROBOT_CODE}/cer-sim" >> ~/.bashrc
    else
        echo "Cer-sim directory not found"
        echo "If you have cer-sim cloned on your machine, please set the CER_SIM_ROOT_DIR manually."
        echo "Otherwise you can clone it by running the following command:"
        echo "git clone https://github.com/robotology/cer-sim.git in a directory of your choice and try again"
    fi
else
    echo "The env is already set to: ${CER_SIM_ROOT_DIR}"
fi

if [ -z "${TOUR_GUIDE_ROBOT_SOURCE_DIR}" ]; then
    if [ -z "${HSP_CODE}" ]; then
        if [ -z "${ROBOT_CODE}" ]; then
            if [ -d "~/robotology/tour-guide-robot" ]; then
                echo "Setting TOUR_GUIDE_ROBOT_SOURCE_DIR to ~/robotology/tour-guide-robot"
                echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=~/robotology/tour-guide-robot" >> ~/.bashrc
                echo "export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/build/share/tour-guide-robot" >> ~/.bashrc
            else
                echo "Tour-guide-robot directory not found"
                echo "If you have tour-guide-robot cloned on your machine, please set the TOUR_GUIDE_ROBOT_SOURCE_DIR manually."
                echo "Otherwise you can clone it by running the following command:"
                echo "git clone https://github.com/hsp-iit/tour-guide-robot.git in a directory of your choice and try again"
            fi
        else
            if [ -d "${ROBOT_CODE}/tour-guide-robot" ]; then
                echo "Setting TOUR_GUIDE_ROBOT_SOURCE_DIR to ${ROBOT_CODE}/tour-guide-robot"
                echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=${ROBOT_CODE}/tour-guide-robot" >> ~/.bashrc
                echo "export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/build/share/tour-guide-robot" >> ~/.bashrc
            else
                echo "Tour-guide-robot directory not found"
                echo "If you have tour-guide-robot cloned on your machine, please set the TOUR_GUIDE_ROBOT_SOURCE_DIR manually."
                echo "Otherwise you can clone it by running the following command:"
                echo "git clone https://github.com/hsp-iit/tour-guide-robot.git in a directory of your choice and try again"
            fi
        fi
    else
        echo "Setting TOUR_GUIDE_ROBOT_SOURCE_DIR to ${HSP_CODE}/tour-guide-robot"
        echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=${HSP_CODE}/tour-guide-robot" >> ~/.bash_profile
        echo "export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/build/share/tour-guide-robot" >> ~/.bashrc
    fi
else
    echo "The env is already set to: ${TOUR_GUIDE_ROBOT_SOURCE_DIR}"
fi

if [[ ":$PATH:" == *":app/headSynchronizer/scripts:"* ]]; then
    echo "Your path is correctly set"
else
    if grep -q "app/headSynchronizer/scripts" ~/.bashrc;
    then
        echo "Your bashrc is set as needed. You just have to source it"
    else
        echo "Your path is missing the path to the joypad scripts. Adding them."
        echo "export PATH=\$PATH:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/scripts:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/headSynchronizer/scripts" >> ~/.bashrc
    fi
fi
