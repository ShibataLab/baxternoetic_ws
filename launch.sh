#!/bin/bash

function help {
  echo "usage: ./launch.sh [--no-graphics]"
  exit 1
}

if [ $# -ne 0 ]; then
  if [ $1 = "--help" ]; then
    help
  fi
  
  if [ $1 = "--no-graphics" ]; then
    echo "Running in no-graphics mode"
    DOCKER_GPU_ARGS=""
    shift
    DOCKER_APP="/bin/bash"
  fi
else
    echo "Running in graphics mode"
    export containerId=$(docker ps -l -q)
    xhost +local:root # to allow GUI
    DOCKER_APP="/entrypoint.sh"
fi

# Settings required for having graphic mode
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --ipc=host --privileged --gpus all"
DOCKER_COMMAND="docker run -it"
DOCKER_NETWORK_ARGS="--net=host"
DOCKER_REALTIME_ARGS="--cap-add=SYS_NICE --ulimit rttime=-1"
DOCKER_REALTIME_PRIO="--ulimit rtprio=99"
DOCKER_REALTIME_MEM="--ulimit memlock=102400:102400"
DOCKER_VOLUME_ARGS="-v $PWD:/home -e HOME=/home -w /home"
DOCKER_IMAGE="kakeru58/baxter_noetic_cuda12.0:latest"
DOCKER_ENTRYPOINT_ARGS="-v $PWD/entrypoint.sh:/entrypoint.sh:ro"

set -x
$DOCKER_COMMAND \
$DOCKER_NETWORK_ARGS \
$DOCKER_REALTIME_ARGS \
$DOCKER_REALTIME_PRIO \
$DOCKER_REALTIME_MEM \
$DOCKER_VOLUME_ARGS \
$DOCKER_ENTRYPOINT_ARGS \
$DOCKER_GPU_ARGS \
$DOCKER_IMAGE \
$DOCKER_APP "$DOCKER_IMAGE"
