FROM konkarapas/r1images:r1Sim2 as ros2CtrlDefault

LABEL maintainer="Ettore Landini"

USER user1
WORKDIR /home/user1

RUN sudo apt-get update && sudo apt-get install -y nautilus

RUN cd tour-guide-robot && git checkout ros2_support && \
    git fetch origin && git pull origin ros2_support && cd build && \
    cmake .. && make -j11

WORKDIR /home/user1/robotology

RUN cd cer-sim && git remote add elandini84 https://github.com/elandini84/cer-sim.git && \
    git fetch elandini84

RUN cd navigation && git remote add elandini84 https://github.com/elandini84/navigation.git && \
    git fetch elandini84 && git checkout --track elandini84/refactor/ros2Localizer && cd build && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cmake .. -DCMAKE_BUILD_TYPE=Release -DNAVIGATION_USE_NWC=ON -DNAVIGATION_USE_ROS2=ON" && \
    make -j11 && echo "source /opt/ros/humble/setup.bash" >> /home/user1/.bashrc

RUN cd yarp-ros2/ && git remote add elandini84 https://github.com/elandini84/yarp-ros2.git && \
     git fetch elandini84 && git checkout --track elandini84/test/ros2_control && cd build && \
      /bin/bash -c "source /opt/ros/humble/setup.bash; cmake .." && make -j11

WORKDIR /home/user1

RUN mkdir /home/user1/ros2_control_ws && cd ros2_control_ws && mkdir src && \
    cd src && git clone https://github.com/ros-controls/ros2_control && \
    git clone https://github.com/ros-controls/ros2_controllers && \
    git clone https://github.com/ros-controls/ros2_control_demos && \
    git clone https://github.com/ros2/ros_testing && cd .. && \
     /bin/bash -c "source /opt/ros/humble/setup.bash; rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install" && \
    echo "source /home/user1/ros2_control_ws/install/setup.bash" >> /home/user1/.bashrc

FROM ros2CtrlDefault as moveit2

RUN sudo apt-get update && sudo apt-get install -y ros-humble-moveit*

RUN sudo apt-get install -y python3-colcon-common-extensions python3-colcon-mixin && \
     /bin/bash -c "source /opt/ros/humble/setup.bash; colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml" && \
     /bin/bash -c "source /opt/ros/humble/setup.bash; colcon mixin update default"

RUN mkdir -p /home/user1/ws_moveit2/src && cd ws_moveit2/src && \
    git clone https://github.com/ros-planning/moveit2_tutorials -b main && \
    vcs import < moveit2_tutorials/moveit2_tutorials.repos && cd .. && \
     /bin/bash -c "source /opt/ros/humble/setup.bash; source /home/user1/ros2_control_ws/install/setup.bash; colcon build --mixin release"
RUN echo "source ~/ws_moveit2/install/setup.bash" >> ~/.bashrc
