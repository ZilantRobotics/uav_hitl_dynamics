#!/bin/bash

print_help() {
   echo "Wrapper under docker API for VTOL dynamics.
It encapsulates all necessary docker flags and properly handles image versions.
https://github.com/RaccoonlabDev/inno_vtol_dynamics

usage: docker.sh [command]

Commands:
build (b)                       Build docker image.
pull                            Pull docker image.
push                            Push docker image.
interactive (i)                 Run container in interactive mode.
run (r)                         Run launch file.
kill                            Kill all containers.
help                            Print this message and exit"
}

DOCKERFILE_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
DOCKERFILE_PATH=$DOCKERFILE_DIR/Dockerfile

setup_image_name_and_version() {
    TAG_NAME=v0.2.0
    DOCKERHUB_REPOSITOTY=ponomarevda/inno_vtol_dynamics

    if uname -m | grep -q 'aarch64'; then
        TAG_NAME="$TAG_NAME""arm64"
    elif uname -m | grep -q 'x86_64'; then
        TAG_NAME="$TAG_NAME""amd64"
    else
        echo "unknown architecture"
        exit
    fi
    DOCKER_CONTAINER_NAME=$DOCKERHUB_REPOSITOTY:$TAG_NAME
}

setup_config() {
    setup_image_name_and_version
    DOCKER_FLAGS="--net=host"
    DOCKER_FLAGS="$DOCKER_FLAGS -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1)"
}

build_docker_image() {
    setup_image_name_and_version
    docker build -f $DOCKERFILE_PATH -t $DOCKER_CONTAINER_NAME ..
}

pull_docker_image() {
    setup_image_name_and_version
    docker pull $DOCKER_CONTAINER_NAME
}

push_docker_image() {
    setup_image_name_and_version
    docker push $DOCKER_CONTAINER_NAME
}

run_interactive() {
    setup_config
    docker container run --rm -it $DOCKER_FLAGS $DOCKER_CONTAINER_NAME /bin/bash
}

run() {
    setup_config
    docker container run --rm $DOCKER_FLAGS $DOCKER_CONTAINER_NAME
}

kill_all_containers() {
    docker kill $(docker ps -q)
}

cd "$(dirname "$0")"

if [ "$1" = "build" ] || [ "$1" = "b" ]; then
    build_docker_image
elif [ "$1" = "pull" ]; then
    pull_docker_image
elif [ "$1" = "push" ]; then
    push_docker_image
elif [ "$1" = "interactive" ] || [ "$1" = "i" ]; then
    run_interactive
elif [ "$1" = "run" ] || [ "$1" = "r" ]; then
    run
elif [ "$1" = "kill" ]; then
    kill_all_containers
else
    print_help
fi
