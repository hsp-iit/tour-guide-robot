# Default values
ros_distro=$ROS_DISTRO
r1ObrRemote=$R1_OBR_REMOTE
r1ObrBranch=$R1_OBR_BRANCH
tourRemote=$TOUR_REMOTE
tourBranch=$TOUR_BRANCH
ROBOT_CODE=/usr/local/src/robot

cd ${ROBOT_CODE}/tour-guide-robot/ && rm -rf build && mkdir build && \
    (git fetch ${tourRemote} || (git remote add ${tourRemote} https://github.com/${tourRemote}/tour-guide-robot && git fetch ${tourRemote})) && git checkout ${tourBranch} && git pull && cd build && cmake .. -DENABLE_faceExpressionImage=ON -DSILERO_VAD=ON && make -j11
cd ${ROBOT_CODE}/r1-object-retrieval/ && rm -rf build && mkdir build && \
    (git fetch ${r1ObrRemote} || (git remote add ${r1ObrRemote} https://github.com/${r1ObrRemote}/r1-object-retrieval && git fetch ${r1ObrRemote})) && git checkout ${r1ObrBranch} && git pull && cd build && cmake .. && make -j11
exec "$@"
