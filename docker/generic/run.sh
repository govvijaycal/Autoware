#!/bin/bash

usage() { echo "Usage: $0 [-s <Shared directory>]" 1>&2; exit 1; }

# Defaults
XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/autoware/shared_dir
HOST_DIR=/home/$USER/shared_dir
DOCKER_HUB_REPO="autoware-kinetic"
TAG="latest"

while getopts ":ht:r:s:" opt; do
  case $opt in
    h)
      usage
      exit
      ;;
    s)
      HOST_DIR=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

echo "Using $DOCKER_HUB_REPO:$TAG"
echo "Shared directory: ${HOST_DIR}"

docker run \
    --gpus all\
    -it --rm \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    -u autoware \
    --privileged -v /dev/bus/usb:/dev/bus/usb \
    --net=host \
    $DOCKER_HUB_REPO:$TAG
