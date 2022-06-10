sudo xhost +
FILE=~/.bashrc_local
if [ -f "$FILE" ]; then
  sudo docker run --rm -it --network host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ~/.bashrc_local:/home/user1/.bashrc_local -v /etc/hosts:/etc/hosts -e QT_X11_NO_MITSHM=1 --gpus all konkarapas/r1images:r1Sim2
else
  sudo docker run --rm -it --network host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}/.bashrc_local:/home/user1/.bashrc_local -e QT_X11_NO_MITSHM=1 --gpus all konkarapas/r1images:r1Sim2
fi
