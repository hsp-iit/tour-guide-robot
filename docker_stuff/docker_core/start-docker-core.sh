sudo xhost +
FILE=~/.bashrc_local
if [ -f "$FILE" ]; then
  sudo docker run --rm -it --privileged --network host --pid=host --device /dev/snd -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native -v ~/.config/pulse/cookie:/root/.config/pulse/cookie -v ~/.bashrc_local:/home/user1/.bashrc_local -v /etc/hosts:/etc/hosts --group-add $(getent group audio | cut -d: -f3) -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all konkarapas/r1images:r1Core
else
  sudo docker run --rm -it --privileged --network host --pid=host --device /dev/snd -e PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native -v ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native -v ~/.config/pulse/cookie:/root/.config/pulse/cookie -v ${PWD}/.bashrc_local:/home/user1/.bashrc_local --group-add $(getent group audio | cut -d: -f3) -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all konkarapas/r1images:r1Core
fi
