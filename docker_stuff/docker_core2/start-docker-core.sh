sudo xhost +
FILE=~/.bashrc_local
if [ -f "$FILE" ]; then
  sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /etc/hosts:/etc/hosts -e QT_X11_NO_MITSHM=1 --gpus all konkarapas/r1images:r1Core2
else
  sudo docker run --rm -it --privileged --network host --pid host -e NVIDIA_DRIVER_CAPABILITIES=all -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1 --gpus all konkarapas/r1images:r1Core2
fi
