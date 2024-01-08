#!/bin/bash

echo "Starting DevCon from context: $PWD"

docker run -it --rm \
    --privileged \
    --net host \
    --volume ${PWD}:/workspace \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --env DISPLAY=$DISPLAY \
    --name kube-bullet-devcon \
    metagoto/kube-bullet-devcon:latest \
    $*