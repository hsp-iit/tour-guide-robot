# Default values
ros_distro=$ROS_DISTRO
r1ObrRemote=$R1_OBR_REMOTE
r1ObrBranch=$R1_OBR_BRANCH

. /opt/ros/${ros_distro}/setup.sh
. ${ROBOT_CODE}/yarp-devices-ros2/ros2_interfaces_ws/install/setup.sh
cd ${ROBOT_CODE}/ycm && git pull && cd build && cmake .. && make -j11 && \
    cd ${ROBOT_CODE}/yarp && git fetch origin && git checkout yarp-3.12 && git pull && cd build && cmake .. && make -j11 && \
    cd ${ROBOT_CODE}/icub-main && git pull && cd build && cmake .. && make -j11 && \
    cd ${ROBOT_CODE}/navigation && git pull && cd build && cmake .. && make -j11 && \
    cd ${ROBOT_CODE}/robots-configuration && git pull && \
    cd ${ROBOT_CODE}/cer/ && git pull && cd build && cmake .. && make -j11 && \
    cd ${ROBOT_CODE}/tour-guide-robot/ && git pull && cd build && cmake .. && make -j11 && \
    cd ${ROBOT_CODE}/r1-object-retrieval/ && rm stop_sim.sh && rm -rf build && mkdir build && \
        (git fetch ${r1ObrRemote} || (git remote add ${r1ObrRemote} https://github.com/${r1ObrRemote}/r1-object-retrieval && git fetch ${r1ObrRemote})) && git checkout ${r1ObrBranch} && git pull && cd build && cmake .. && make -j11
exec "$@"