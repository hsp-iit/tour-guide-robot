if [ -z "${TOUR_GUIDE_ROBOT_SOURCE_DIR}" ]; then
    if [ -z "${HSP_CODE}" ]; then
        echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=~/tour-guide-robot" >> ~/.bashrc
    else
        echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=${HSP_CODE}/tour-guide-robot" >> ~/.bash_profile
    fi
else
    echo "The env is already set to: ${TOUR_GUIDE_ROBOT_SOURCE_DIR}"
fi

if [ -z "${CER_SIM_ROOT_DIR}" ]; then
    if [ -z "${ROBOT_CODE}" ]; then
        echo "export CER_SIM_ROOT_DIR=~/robotology/cer-sim" >> ~/.bashrc
    else
        echo "export CER_SIM_ROOT_DIR=${ROBOT_CODE}/cer-sim" >> ~/.bash_profile
    fi
else
    echo "The env is already set to: ${CER_SIM_ROOT_DIR}"
fi
