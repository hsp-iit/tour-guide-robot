if [ -z "${TOUR_GUIDE_ROBOT_SOURCE_DIR}" ]; then
    if [ -z "${HSP_CODE}" ]; then
        echo "Setting TOUR_GUIDE_ROBOT_SOURCE_DIR to ~/tour-guide-robot"
        echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=~/tour-guide-robot" >> ~/.bashrc
    else
        echo "Setting TOUR_GUIDE_ROBOT_SOURCE_DIR to ${HSP_CODE}/tour-guide-robot"
        echo "export TOUR_GUIDE_ROBOT_SOURCE_DIR=${HSP_CODE}/tour-guide-robot" >> ~/.bash_profile
    fi
else
    echo "The env is already set to: ${TOUR_GUIDE_ROBOT_SOURCE_DIR}"
fi

if [ -z "${CER_SIM_ROOT_DIR}" ]; then
    if [ -z "${ROBOT_CODE}" ]; then
        echo "Setting CER_SIM_ROOT_DIR to ~/robotology/cer-sim"
        echo "export CER_SIM_ROOT_DIR=~/robotology/cer-sim" >> ~/.bashrc
    else
        echo "Setting CER_SIM_ROOT_DIR to ${ROBOT_CODE}/cer-sim"
        echo "export CER_SIM_ROOT_DIR=${ROBOT_CODE}/cer-sim" >> ~/.bashrc
    fi
else
    echo "The env is already set to: ${CER_SIM_ROOT_DIR}"
fi
if [[ ":$PATH:" == *":app/headSynchronizer/scripts:"* ]]; then
  echo "Your path is correctly set"
else
  echo "Your path is missing the path to the joypad scripts. Adding them."
  echo "export PATH=\$PATH:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/navigation2/scripts:${TOUR_GUIDE_ROBOT_SOURCE_DIR}/app/headSynchronizer/scripts" >> ~/.bashrc
fi
